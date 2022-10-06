import copy
import logging
import os
import time
import traceback

from typing import Tuple, List
import warnings

from commonroad.common.solution import PlanningProblemSolution, Solution
from SMP.solution import CommonRoadSearchSolutionWriter
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import State, Trajectory
from commonroad_dc.feasibility.solution_checker import valid_solution

import SMP.batch_processing.helper_functions as hf
from SMP.batch_processing.scenario_loader import ScenarioLoader, ScenarioConfig
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from SMP.motion_planner.motion_planner import MotionPlanner, MotionPlannerType
from SMP.motion_planner.plot_config import StudentScriptPlotConfig
from SMP.batch_processing.helper_functions import ResultType, ResultText, SearchResult, call_subprocess


def get_planning_problem_and_id(planning_problem_set: PlanningProblemSet, planning_problem_idx) -> \
        Tuple[PlanningProblem, int]:
    return list(planning_problem_set.planning_problem_dict.values())[planning_problem_idx], \
           list(planning_problem_set.planning_problem_dict.keys())[planning_problem_idx]


def solve_scenario(scenario, planning_problem, automaton, config: ScenarioConfig, result_dict) -> SearchResult:
    scenario_id = str(scenario.scenario_id)

    error_msg = ""
    list_of_list_of_states = None

    def get_search_time_in_sec(start_time):
        return time.perf_counter() - start_time

    def get_search_time_in_ms(start_time):
        return get_search_time_in_sec(start_time) * 1000

    time1 = time.perf_counter()

    try:
        motion_planner = MotionPlanner.create(scenario, planning_problem, automaton=automaton,
                                              plot_config=StudentScriptPlotConfig(DO_PLOT=False),
                                              motion_planner_type=config.motion_planner_type)
        list_of_list_of_states, list_of_motion_primitives, _ = motion_planner.execute_search()
    except Exception as err:
        # TODO consider giving -1 back because evaluating it out with excel then will be easier
        search_time_ms = get_search_time_in_ms(time1)
        error_msg = "".join(traceback.format_exception(type(err), err, err.__traceback__))

        result = ResultType.EXCEPTION
    else:
        if list_of_list_of_states is None:
            search_time_ms = get_search_time_in_ms(time1)
            result = ResultType.FAILURE
        else:
            search_time_ms = get_search_time_in_ms(time1)
            result = ResultType.SUCCESS
    res = SearchResult(scenario_id, result, search_time_ms, config.motion_planner_type, error_msg,
                       list_of_list_of_states)
    if not (result_dict is None):
        result_dict[scenario_id] = res

    return res


def validate_solution_before_saving(scenario: Scenario, planning_problem_set: PlanningProblemSet,
                                    planning_problem_id: int,
                                    config: ScenarioConfig,
                                    computation_time_in_sec: float, list_of_list_of_states: List[List[State]],
                                    validate_solution: bool = True,
                                    logger: logging.Logger = logging.getLogger()) -> bool:
    # create solution object for benchmark
    pps = PlanningProblemSolution(planning_problem_id=planning_problem_id,
                                  vehicle_type=config.vehicle_type,
                                  vehicle_model=config.vehicle_model,
                                  cost_function=config.cost_function,
                                  trajectory=SearchResult.compute_solution_trajectory(list_of_list_of_states))

    solution = Solution(scenario.scenario_id, [pps], computation_time=computation_time_in_sec)

    if validate_solution:
        if valid_solution(scenario, planning_problem_set, solution)[0]:
            solution_valid = True
        else:
            solution_valid = False
    else:
        solution_valid = True

    return (solution_valid, pps, solution)


def save_validated_solution(solution: Solution, planning_problem_solution: PlanningProblemSolution, scenario: Scenario,
                            planning_problem_set: PlanningProblemSet, planning_problem_id: int,
                            output_path: str = './',
                            overwrite: bool = False, save_gif: bool = False,
                            output_path_gif: str = './gifs', logger: logging.Logger = logging.getLogger()):
    # write solution to a xml file
    csw = CommonRoadSearchSolutionWriter(solution)
    if save_gif:
        # create directory if it does not exist
        os.makedirs(output_path_gif, exist_ok=True)
        hf.save_gif2(scenario, planning_problem_set.find_planning_problem_by_id(planning_problem_id),
                     planning_problem_solution.trajectory,
                     output_path=output_path_gif)

    # create directory if not exists
    os.makedirs(output_path, exist_ok=True)

    csw.write_to_file(output_path=output_path, overwrite=overwrite)

    return True


def append_element_2_list_in_dict(the_dict, key, new_element, immutable_dictionary: bool = True):
    if immutable_dictionary:
        foo_list = the_dict[key]
        foo_list.append(new_element)
        the_dict[key] = foo_list
    else:
        the_dict[key].append(new_element)


def postprocess_result(result, result_dict, scenario_id, scenario_loader, configuration_dict,
                       logger: logging.Logger = logging.getLogger(), verbose=False):
    scenario_config = ScenarioConfig(scenario_id, configuration_dict)
    # Loading Scenario and Planning Problem Set
    scenario, planning_problem_set = scenario_loader.load_scenario(scenario_id)

    # Retrieve Planning Problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
    # with the GSMP approach we do not want to solve cooperative scenarios so in all cases we will have
    # only one planning problem

    planning_problem, planning_problem_id = get_planning_problem_and_id(planning_problem_set,
                                                                        scenario_config.planning_problem_idx)
    if isinstance(result, SearchResult):

        search_result: SearchResult = result

        result_dict[scenario_id] = result

        if search_result.result == ResultType.TIMEOUT:
            if verbose:
                print(
                    f"\n{scenario_id:<25}  {search_result.result}  Timeout time [s]:  {int(scenario_config.timeout)}")
            append_element_2_list_in_dict(result_dict, ResultType.TIMEOUT, scenario_id, immutable_dictionary=False)

        elif search_result.result == ResultType.EXCEPTION:
            if verbose:
                print(f"\n{scenario_id:<25}  {search_result.result}  {search_result.error_msg}")
            append_element_2_list_in_dict(result_dict, ResultType.EXCEPTION, scenario_id, immutable_dictionary=False)

        elif search_result.result == ResultType.FAILURE:
            if verbose:
                print(
                    f"\n{scenario_id:<25}  {search_result.result}  Computation time [ms]:  {int(search_result.search_time_ms)}"
                    f"  <{scenario_config.motion_planner_type}>  DID NOT FIND a solution.")
            append_element_2_list_in_dict(result_dict, ResultType.FAILURE, scenario_id, immutable_dictionary=False)

        else:
            if verbose:
                print(
                    f"\n{scenario_id:<25}  {search_result.result}  Computation time [ms]:  {int(search_result.search_time_ms)}  "
                    f"<{scenario_config.motion_planner_type}>  FOUND a solution.")

            is_valid_solution = (search_result.result == ResultType.SUCCESS)
            if is_valid_solution:
                append_element_2_list_in_dict(result_dict, ResultType.SUCCESS, scenario_id, immutable_dictionary=False)
            else:
                # replacing the search result in the statistics
                tmp_sr = result_dict[scenario_id]
                result_dict[scenario_id] = SearchResult(
                    scenario_benchmark_id=scenario_id,
                    result=ResultType.INVALID_SOLUTION,
                    search_time_ms=tmp_sr.search_time_ms,
                    motion_planner_type=scenario_config.motion_planner_type,
                    list_of_list_of_states=tmp_sr.list_of_list_of_states)

                append_element_2_list_in_dict(result_dict, ResultType.INVALID_SOLUTION, scenario_id,
                                              immutable_dictionary=False)


def process_scenario(scenario_id, scenario_loader: ScenarioLoader, configuration_dict, def_automaton: ManeuverAutomaton,
                     exitFlag,
                     logger: logging.Logger = logging.getLogger(),
                     verbose=False):
    warnings.filterwarnings("ignore")
    # noinspection PyBroadException
    try:

        logger.debug("Start processing [{:<30}]".format(scenario_id))

        # Parse configuration dict
        scenario_config = ScenarioConfig(scenario_id, configuration_dict)

        # AUTOMATON preparation
        if def_automaton.type_vehicle != scenario_config.vehicle_type:
            # if the defined vehicle type differs from the default one then load custom automaton instead
            try:
                automaton = ManeuverAutomaton.load_automaton(
                    hf.get_default_automaton_by_veh_id(scenario_config.vehicle_type_id, configuration_dict))
            except FileNotFoundError:
                try:
                    automaton = ManeuverAutomaton.generate_automaton(
                        hf.get_default_motion_primitive_file_by_veh_id(scenario_config.vehicle_type_id,
                                                                       configuration_dict))
                except FileNotFoundError:
                    raise FileNotFoundError(
                        f"No default MotionAutomaton found for vehicle type id: {scenario_config.vehicle_type}")
        else:
            # use the default automaton file if there is nothing else specified
            automaton = copy.deepcopy(def_automaton)
            automaton.deserialize()

        # Loading Scenario and Planning Problem Set
        scenario, planning_problem_set = scenario_loader.load_scenario(scenario_id)

        # Retrieve Planning Problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
        # with the GSMP approach we do not want to solve cooperative scenarios so in all cases we will have
        # only one planning problem
        planning_problem, planning_problem_id = get_planning_problem_and_id(planning_problem_set,
                                                                            scenario_config.planning_problem_idx)

        success, res_local = call_subprocess(solve_scenario,
                                             args=(scenario, planning_problem, automaton, scenario_config, None),
                                             exitFlag=exitFlag, timeout=scenario_config.timeout)

        if success == ResultType.EXCEPTION:
            return SearchResult(scenario_id, ResultType.EXCEPTION, scenario_config.timeout,
                                scenario_config.motion_planner_type)
        if success == ResultType.TIMEOUT:
            return SearchResult(scenario_id, ResultType.TIMEOUT, scenario_config.timeout,
                                scenario_config.motion_planner_type)

        if (
                res_local.result != ResultType.FAILURE and res_local.result != ResultType.TIMEOUT and res_local.result != ResultType.EXCEPTION):

            # validate solution, save it only if it is valid

            validate_solution = hf.str2bool(
                configuration_dict['setting']['validate_solution'])

            success, validation_result = call_subprocess(validate_solution_before_saving,
                                                         args=(scenario, planning_problem_set, planning_problem_id,
                                                               scenario_config, res_local.search_time_sec,
                                                               res_local.list_of_list_of_states, validate_solution,
                                                               logger),
                                                         exitFlag=exitFlag, timeout=None)

            if success == ResultType.EXCEPTION:
                return SearchResult(scenario_id, ResultType.EXCEPTION, scenario_config.timeout,
                                    scenario_config.motion_planner_type)

            is_valid_solution, planning_problem_solution, solution = validation_result

            if is_valid_solution:
                save_validated_solution(planning_problem_solution=planning_problem_solution, solution=solution,
                                        scenario=scenario, planning_problem_set=planning_problem_set,
                                        planning_problem_id=planning_problem_id,
                                        output_path=configuration_dict['setting']['output_path'],
                                        overwrite=hf.str2bool(configuration_dict['setting']['overwrite']),
                                        save_gif=hf.str2bool(
                                            configuration_dict['setting']['create_gif']),
                                        output_path_gif=configuration_dict['setting']['output_path_gif'],
                                        logger=logger)
                res_local.result = ResultType.SUCCESS
            else:
                res_local.result = ResultType.INVALID_SOLUTION
    except Exception as err:
        print("Something went wrong while processing scenario {:<30}]".format(scenario_id))
        error_msg = "".join(traceback.format_exception(type(err), err, err.__traceback__))
        print(error_msg)
        return SearchResult(scenario_id, ResultType.EXCEPTION, scenario_config.timeout,
                            scenario_config.motion_planner_type)
    return res_local


def debug_scenario(scenario_id, scenario_loader: ScenarioLoader, configuration_dict, def_automaton: ManeuverAutomaton,
                   logger: logging.Logger = logging.getLogger(),
                   verbose=False):
    warnings.filterwarnings("ignore")
    # noinspection PyBroadException
    try:

        logger.debug("Start processing [{:<30}]".format(scenario_id))

        # Parse configuration dict
        scenario_config = ScenarioConfig(scenario_id, configuration_dict)

        # AUTOMATON preparation
        if def_automaton.type_vehicle != scenario_config.vehicle_type:
            # if the defined vehicle type differs from the default one then load custom automaton instead
            try:
                automaton = ManeuverAutomaton.load_automaton(
                    hf.get_default_automaton_by_veh_id(scenario_config.vehicle_type_id, configuration_dict))
            except FileNotFoundError:
                try:
                    automaton = ManeuverAutomaton.generate_automaton(
                        hf.get_default_motion_primitive_file_by_veh_id(scenario_config.vehicle_type_id,
                                                                       configuration_dict))
                except FileNotFoundError:
                    raise FileNotFoundError(
                        f"No default MotionAutomaton found for vehicle type id: {scenario_config.vehicle_type}")
        else:
            # use the default automaton file if there is nothing else specified
            automaton = copy.deepcopy(def_automaton)
            automaton.deserialize()

        # Loading Scenario and Planning Problem Set
        scenario, planning_problem_set = scenario_loader.load_scenario(scenario_id)

        # Retrieve Planning Problem with given index (for cooperative scenario:0, 1, 2, ..., otherwise: 0)
        # with the GSMP approach we do not want to solve cooperative scenarios so in all cases we will have
        # only one planning problem
        planning_problem, planning_problem_id = get_planning_problem_and_id(planning_problem_set,
                                                                            scenario_config.planning_problem_idx)

        res_local = solve_scenario(scenario, planning_problem, automaton, scenario_config, None)

        if (
                res_local.result != ResultType.FAILURE and res_local.result != ResultType.TIMEOUT and res_local.result != ResultType.EXCEPTION):

            # validate solution, save it only if it is valid

            validate_solution = hf.str2bool(
                configuration_dict['setting']['validate_solution'])

            validation_result = validate_solution_before_saving(scenario, planning_problem_set, planning_problem_id,
                                                                scenario_config, res_local.search_time_sec,
                                                                res_local.list_of_list_of_states, validate_solution,
                                                                logger)

            is_valid_solution, planning_problem_solution, solution = validation_result

            if is_valid_solution:
                save_validated_solution(planning_problem_solution=planning_problem_solution, solution=solution,
                                        scenario=scenario, planning_problem_set=planning_problem_set,
                                        planning_problem_id=planning_problem_id,
                                        output_path=configuration_dict['setting']['output_path'],
                                        overwrite=hf.str2bool(configuration_dict['setting']['overwrite']),
                                        save_gif=hf.str2bool(
                                            configuration_dict['setting']['create_gif']),
                                        output_path_gif=configuration_dict['setting']['output_path_gif'],
                                        logger=logger)
                res_local.result = ResultType.SUCCESS
            else:
                res_local.result = ResultType.INVALID_SOLUTION
    except Exception as err:
        print("Something went wrong while processing scenario {:<30}]".format(scenario_id))
        error_msg = "".join(traceback.format_exception(type(err), err, err.__traceback__))
        print(error_msg)
        return SearchResult(scenario_id, ResultType.EXCEPTION, scenario_config.timeout,
                            scenario_config.motion_planner_type)
    return res_local

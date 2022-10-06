import logging
import warnings

from concurrent.futures import ProcessPoolExecutor as Pool
import SMP.batch_processing.helper_functions as hf
from SMP.batch_processing.process_scenario import process_scenario, postprocess_result

import time
from multiprocessing import Manager


def init_parallel_processing():
    exitFlag = Manager().Event()
    exitFlag.clear()
    context = dict()
    context["exitFlag"] = exitFlag
    return context


def stop_parallel_processing(context: dict):
    exitFlag = context["exitFlag"]
    exitFlag.set()
    time.sleep(1)


def run_parallel_processing():
    warnings.filterwarnings("ignore")

    context = init_parallel_processing()

    configuration, logger, scenario_loader, def_automaton, result_dict = hf.init_processing("Batch Processor",
                                                                                            for_multi_processing=False)

    num_worker = hf.parse_processes_number(int(configuration['setting']['num_worker_processes']))

    message = f"Number of parallel processes: {num_worker}"
    logger.info(message)
    print(message)

    logger.setLevel(logging.INFO)

    list_futures = []
    list_args = []
    exitFlag = context["exitFlag"]

    with Pool(max_workers=num_worker) as p:
        try:
            for idx, scenario_id in enumerate(result_dict["scenarios_to_process"]):
                list_futures.append(
                    p.submit(process_scenario, *(scenario_id, scenario_loader, configuration, def_automaton, exitFlag)))
                list_args.append(scenario_id)

            for idx, scenario_id in enumerate(list_args):
                ret_result = list_futures[idx].result()
                list_futures[idx] = None
                postprocess_result(ret_result, result_dict, scenario_id, scenario_loader, configuration)
                result_dict["started_processing"] += 1
                hf.print_current_status(result_dict)

        except KeyboardInterrupt:
            print('stopping execution')
            for future in list_futures:
                if not (future is None):
                    future.cancel()
            stop_parallel_processing(context)

    hf.log_detailed_scenario_results(logger, result_dict)
    hf.log_statistics(logger, result_dict, verbose=True)


if __name__ == '__main__':
    run_parallel_processing()

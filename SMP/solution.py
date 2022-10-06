from commonroad.common.solution import CommonRoadSolutionWriter
import os
import platform


class CommonRoadSearchSolutionWriter(CommonRoadSolutionWriter):
    def __init__(self, solution):
        super().__init__(solution)

    def escape_path(self, path):
        if platform.system() == 'Windows':
            return path.replace(":", ";")
        else:
            return path

    def write_to_file(self, output_path: str = './', filename: str = None,
                      overwrite: bool = False, pretty: bool = True):
        """
        Writes the Solution XML to a file.

        :param output_path: Output dir where the Solution XML file should be written to. \
            Writes to the same folder where it is called from if not specified.
        :param filename: Name of the Solution XML file. If not specified, sets the name as 'solution_BENCHMARKID.xml' \
            where the BENCHMARKID is the benchmark_id of the solution.
        :param overwrite: If set to True, overwrites the file if it already exists.
        :param pretty: If set to True, prettifies the Solution XML string before writing to file.
        """
        filename = filename if filename is not None else 'solution_%s.xml' % self.escape_path(
            self.solution.benchmark_id)
        fullpath = os.path.join(output_path, filename) if filename is not None else os.path.join(output_path, filename)

        if not os.path.exists(os.path.dirname(fullpath)):
            raise NotADirectoryError("Directory %s does not exist." % os.path.dirname(fullpath))

        if os.path.exists(fullpath) and not overwrite:
            raise FileExistsError("File %s already exists. If you want to overwrite it set overwrite=True." % fullpath)

        with open(fullpath, 'w') as f:
            f.write(self.dump(pretty))

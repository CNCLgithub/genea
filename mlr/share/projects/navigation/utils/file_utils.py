import csv
import os
import re
import shutil

from mlr.share.projects.navigation.utils.msg_utils import Msg


class FileUtils:
    @staticmethod
    def is_file(filepath):
        return os.path.isfile(filepath)

    @staticmethod
    def is_dir(dirpath):
        return os.path.isdir(dirpath)

    @staticmethod
    def create_dir(dirpath, do_force_create=False):
        if os.path.isdir(dirpath):
            if do_force_create:
                shutil.rmtree(dirpath)
                os.makedirs(dirpath)
            return

        os.makedirs(dirpath)

    @staticmethod
    def create_file(out_filepath):
        FileUtils.create_dir(os.path.dirname(out_filepath))
        file = open(out_filepath, 'wb')
        file.close()

    @staticmethod
    def delete_file(filepath):
        if os.path.isfile(filepath):
            os.remove(filepath)

    @staticmethod
    def copy_file(src_filepath, dest_filepath):
        shutil.copy2(src_filepath, dest_filepath)

    @staticmethod
    def write_to_file(out_filepath, file_contents):
        if not os.path.isfile(out_filepath):
            FileUtils.create_file(out_filepath)

        with open(out_filepath, "w") as file:
            file.write(file_contents)

    @staticmethod
    def write_row_to_file(out_file_path, row_data):
        """
        Writes a new row into the given file
        :param out_file_path: name of the file that is to be written in
        :param row_data: the entire row entry that is to be written into the file
        :return: None
        """
        if not os.path.isfile(out_file_path):
            FileUtils.create_file(out_file_path)

        try:
            file = open(out_file_path, 'a')
            writer = csv.writer(file)
            writer.writerow(row_data)
            file.close()

        except Exception as e:
            Msg.print_error("ERROR [write_row_to_file]: could not write into " + out_file_path)
            Msg.print_error(str(e))
            assert False

    @staticmethod
    def get_file_basename(filepath, is_attached=True):
        if is_attached:
            return os.path.basename(filepath)
        return os.path.splitext(os.path.basename(filepath))

    @staticmethod
    def get_dir_list_in_directory(dirpath):
        try:
            return list(filter(os.path.isdir, [os.path.join(dirpath, i) for i in os.listdir(dirpath)]))
        except Exception as e:
            Msg.print_error("ERROR: issue in retrieving directory list within given directory " + dirpath)
            Msg.print_error(str(e))

    @staticmethod
    def get_files_in_directory(dirpath, is_sorted=True, is_recursive=False):
        try:
            file_paths_list = list(filter(
                os.path.isfile, [os.path.join(dirpath, i) for i in os.listdir(dirpath) if not i.startswith('.')])
            )

            if is_recursive:
                for subdir in FileUtils.get_dir_list_in_directory(dirpath):
                    file_paths_list.extend(FileUtils.get_files_in_directory(subdir, is_sorted, True))

            if not is_sorted:
                return file_paths_list
            return sorted(file_paths_list, key=lambda f: [int(n) for n in re.findall(r"\d+", f)])
        except Exception as e:
            Msg.print_error("ERROR: issue in retrieving files within directory " + dirpath)
            Msg.print_error(str(e))

    @staticmethod
    def read_csv_file(filepath, delimiter=","):
        """
        Reads and returns the data within the given file,
        or returns with an error message if no such file exists
        :param filepath    : name of the file that is to be read
        :param delimiter    : the type of the delimiter
        :return: a list of list containing all the data
        """

        try:
            file = open(filepath, 'r')
            reader = csv.reader(file, delimiter=delimiter)

            data = []
            for row in reader:
                data.append(row)
            file.close()

            return data

        except Exception:
            Msg.print_error("Error while reading from " + filepath)
            raise FileNotFoundError

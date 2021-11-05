import os
from datetime import datetime
import csv

from globals import PATH_TO_EXPERIMENT_RECORDINGS

try:
    # Use gitpython to get a current revision number and use it in description of experimental data
    from git import Repo
except ModuleNotFoundError:
    print('GitPython not found')

def csv_init(csv_name=None, controller_name=None):

    # Make folder to save data (if not yet existing)
    try:
        os.makedirs(PATH_TO_EXPERIMENT_RECORDINGS[:-1])
    except FileExistsError:
        pass

    # Set path where to save the data
    if csv_name is None or csv_name == '':
        if controller_name is not None:
            csv_filepath = PATH_TO_EXPERIMENT_RECORDINGS + 'CP_' + controller_name + str(
                datetime.now().strftime('_%Y-%m-%d_%H-%M-%S')) + '.csv'
        else:
            # This is the original version from Tobi
            csv_filepath = PATH_TO_EXPERIMENT_RECORDINGS + datetime.now().strftime("cartpole-%Y-%m-%d-%H-%M-%S.csv")
    else:
        csv_filepath = PATH_TO_EXPERIMENT_RECORDINGS + csv_name
        if csv_name[-4:] != '.csv':
            csv_filepath += '.csv'

        # If such file exists, append index to the end (do not overwrite)
        net_index = 1
        logpath_new = csv_filepath
        while True:
            if os.path.isfile(logpath_new):
                logpath_new = csv_filepath[:-4]
            else:
                csv_filepath = logpath_new
                break
            logpath_new = logpath_new + '-' + str(net_index) + '.csv'
            net_index += 1

    # Write the .csv file
    with open(csv_filepath, "a") as outfile:
        writer = csv.writer(outfile)

        writer.writerow(['# ' + 'This is CartPole simulation from {} at time {}'
                        .format(datetime.now().strftime('%d.%m.%Y'), datetime.now().strftime('%H:%M:%S'))])
        try:
            repo = Repo(search_parent_directories=True)
            git_revision = repo.head.object.hexsha
        except NameError:
            git_revision = 'unknown'
        writer.writerow(['# ' + 'Done with git-revision: {}'
                        .format(git_revision)])

        writer.writerow(['#'])

        writer.writerow(['# Units:'])
        writer.writerow(['# time: s'])
        writer.writerow(['# deltaTimeMs: ms'])
        writer.writerow(['# angle_raw'])
        writer.writerow(['# angle: rad'])
        writer.writerow(['# angleD: rad/s'])
        writer.writerow(['# angle_cos:'])
        writer.writerow(['# angle_sin:'])
        writer.writerow(['# position_raw'])
        writer.writerow(['# position: m'])
        writer.writerow(['# positionD: m/s'])
        writer.writerow(['# angleTarget: rad'])
        writer.writerow(['# angleErr: rad'])
        writer.writerow(['# target_position: m'])
        writer.writerow(['# positionErr: m'])
        writer.writerow(['# angleCmd:'])
        writer.writerow(['# positionCmd:'])
        writer.writerow(['# actualMotorSave:'])
        writer.writerow(['# Q: normed motor power'])
        writer.writerow(['# stickControl'])
        writer.writerow(['# stickPos'])
        writer.writerow(['# measurement'])
        writer.writerow(['# angle_squared'])
        writer.writerow(['# position_squared'])
        writer.writerow(['# Q_squared'])
        writer.writerow(['# sent'])
        writer.writerow(['# received'])
        writer.writerow(['# latency'])
        writer.writerow(['# pythonLatency'])
        writer.writerow(['#'])
        writer.writerow(['#'])

        writer.writerow(
            ['time'] + ['deltaTimeMs'] + ['angle_raw'] + ['angle'] + ['angleD'] + ['angle_cos'] + ['angle_sin'] + ['position_raw'] + ['position'] + [
                'positionD'] + ['angleTarget'] + ['angleErr'] + ['target_position'] + ['positionErr'] + ['angleCmd'] + [
                'positionCmd'] + ['actualMotorSave'] + ['Q'] + ['stickControl'] + ['stickPos'] + ['measurement'] + ['angle_squared'] + ['position_squared'] + ['Q_squared'] + ['sent'] + ['received'] + ['latency']  + ['pythonLatency'] + ['additional_latency'])

    # TODO: Not sure if we really need to return these two things and if this is efficient implementation
    csvfile = open(csv_filepath, 'a', newline='')
    csvwriter = csv.writer(csvfile, delimiter=',')

    return csv_filepath, csvfile, csvwriter
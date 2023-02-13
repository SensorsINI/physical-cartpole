import os
from datetime import datetime
import csv

from globals import PATH_TO_EXPERIMENT_RECORDINGS, CONTROL_PERIOD_MS

try:
    # Use gitpython to get a current revision number and use it in description of experimental data
    from git import Repo
except ModuleNotFoundError:
    print('GitPython not found')

def csv_init(csv_name=None, controller_name=None, header_line=None)->None:
    """ Initializes the CSV file to hold saved data
    :param csv_name: the filename
    :param controller_name: the controller name
    :param header_line: the header line of comma-separated column names, ignored if None

    """

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

        writer.writerow(['# ' + 'This is CartPole data from {} at time {}'
                        .format(datetime.now().strftime('%d.%m.%Y'), datetime.now().strftime('%H:%M:%S'))])
        try:
            repo = Repo(search_parent_directories=True)
            git_revision = repo.head.object.hexsha
        except NameError:
            git_revision = 'unknown'
        writer.writerow(['# ' + 'Done with git-revision: {}'
                        .format(git_revision)])

        writer.writerow(['#'])

        writer.writerow(['#'])
        writer.writerow(['# Time intervals dt:'])
        writer.writerow(['# Simulation: 0.002 s'])
        writer.writerow(['# Controller update: '+str(CONTROL_PERIOD_MS/1000)+' s'])
        writer.writerow(['# Saving: '+str(CONTROL_PERIOD_MS/1000)+' s'])
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
        writer.writerow(['# latency'])
        writer.writerow(['# pythonLatency'])
        writer.writerow(['# additionalLatency'])
        writer.writerow(['# invalid_steps'])
        writer.writerow(['# frozen'])
        writer.writerow(['# angleD fitted'])
        writer.writerow(['# predicted next state'])
        writer.writerow(['# target next state'])
        writer.writerow(['#'])
        writer.writerow(['#'])

        if header_line:
            writer.writerow(header_line)
        else:
            writer.writerow(
                ['time'] + ['deltaTimeMs'] + ['angle_raw'] + ['angleD_raw'] + ['angle'] + ['angleD'] + ['angle_cos'] + ['angle_sin'] + ['position_raw'] + ['position'] + [
                    'positionD'] + ['angleTarget'] + ['angleErr'] + ['target_position'] + ['target_equilibrium'] + ['positionErr'] + ['angleCmd'] + [
                    'positionCmd'] + ['actualMotorSave'] + ['Q'] + ['stickControl'] + ['stickPos'] + ['measurement'] + ['angle_squared'] + ['position_squared'] + ['Q_squared'] + ['sent'] + ['latency'] + ['pythonLatency']+ ['controller_steptime'] + ['additionalLatency']
                + ['invalid_steps'] + ['frozen'] + ['fitted'] + ['angle_raw_sensor'] + ['angleD_raw_sensor'] + ['angleD_fitted']
                +['predict_angle'] + ['predict_angleD'] + ['predict_angle_cos'] + ['predict_angle_sin'] + ['predict_position'] + ['predict_positionD']
                +['traj_angle'] + ['traj_angleD'] + ['traj_angle_cos'] + ['traj_angle_sin'] + ['traj_position'] + ['traj_positionD']
        )

    # TODO: Not sure if we really need to return these two things and if this is efficient implementation
    csvfile = open(csv_filepath, 'a', newline='')
    csvwriter = csv.writer(csvfile, delimiter=',')

    return csv_filepath, csvfile, csvwriter
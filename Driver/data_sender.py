from multiprocessing.connection import Client
import time
import pandas as pd


def main():
    # Create a sample DataFrame
    time.sleep(3)
    path = './ExperimentRecordings/CPP_mpc__2024-07-01_00-52-52.csv'
    df = pd.read_csv(path, comment='#')


    df = df[['angle', 'angleD', 'position', 'positionD']]

    # Use the server's local IP address
    address = ('localhost', 6000)  # Replace with the server's local IP address if necessary
    conn = Client(address)

    try:
        # Send the header
        header = df.columns.tolist()
        conn.send(header)
        time.sleep(0.1)

        # Send data line by line
        for _, row in df.iterrows():
            conn.send(row.values)
            time.sleep(0.02)  # Wait for a short time before sending the next row

        # Optionally, send a 'complete' message to indicate end of data
        conn.send('complete')

    finally:
        conn.close()  # Close the connection


if __name__ == '__main__':
    main()

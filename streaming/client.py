import socket
import pickle
import time
import numpy as np

HEADERSIZE = 10

id = 0

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('127.0.0.1', 1245))

full_msg = b''
new_msg = True
wait_response=False

start_time = 0
end_time = 0

while True:

    if not wait_response:
        id += 1
        data = np.random.normal(size=(3,4))
        msg = pickle.dumps({'id':id, 'command':data})
        print(f'-> [{id}] data send.')
        msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8')+msg
        s.sendall(msg)
        wait_response = True

        start_time = time.time()

    else:
        while True:
            msg = s.recv(16)
            if new_msg:
                msglen = int(msg[:HEADERSIZE])
                new_msg = False

            full_msg += msg

            if len(full_msg)-HEADERSIZE == msglen:
                reply = True
                data = pickle.loads(full_msg[HEADERSIZE:])
                data_id = data['id']
                data_obs = data['obs']
                end_time = time.time()
                new_msg = True
                full_msg = b""
                wait_response = False
                print(f"@ Reply Received [{data_id}]:")
                print(f"Ellapsed time: {round(end_time - start_time, 5)} s.")
                break

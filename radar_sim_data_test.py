import socket

radar_sim_ip = "192.168.1.102"
radar_sim_port_num = 55565
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    server_socket.bind((radar_sim_ip, radar_sim_port_num))

except:
    print("Unable to bind to '10.1.10.10' - connecting as localhost")
    server_socket.bind(("", radar_sim_port_num))

while True:
    try:
        raw_data = server_socket.recv(10000)
        print(raw_data)
    except:
        print("did not parse data")
        status = -1

    if raw_data != 0:
        status = 1
        # print(str(int(raw_data[0])) + " / " + str(int(raw_data[1]))

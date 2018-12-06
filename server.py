import socket

HOST = '192.168.1.109'                 # Symbolic name meaning all available interfaces
PORT = 50007              # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(5)

while 1:
    conn, addr = s.accept()
    print 'Connected by', addr
    single = "ecnu"
    data = conn.recv(1024)
    print(data)
    if data.find(single)>=0:
        print ("ok")
        break
conn.close()
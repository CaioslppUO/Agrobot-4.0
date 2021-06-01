import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.bind(('localhost', 3000))
s.listen(1)

conn, addr = s.accept()

data = conn.recv(10000)
print(str(data.decode("utf-8")))
conn.send("Teste de envio a partir do servidor em python3.".encode("utf-8"))


conn.close()
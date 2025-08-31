from networktables import NetworkTables
import networktables as nt, time

NetworkTables.startServer(listenAddress="192.168.1.236", port=6967)
table = NetworkTables.getTable("Table1")
table.putString("Bob", "Bobber")

while True:
    time.sleep(1)
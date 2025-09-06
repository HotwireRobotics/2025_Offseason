from networktables import NetworkTables
import networktables as nt, time

NetworkTables.startServer(listenAddress="", port=6967)
table = NetworkTables.getTable("Table1")
table.putString("Bob", "Bobber")

while True:
    time.sleep(1)
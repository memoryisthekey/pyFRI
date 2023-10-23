import matplotlib.pyplot as plt
import paho.mqtt.client as mqtt

# MQTT configuration
mqtt_broker_host = '0.0.0.0'
mqtt_force_topic = "forcetorque"

# Variables to store orientation data
data = {"time": [], "fx": [], "fy": [], "fz": [], "tx": [], "ty": [], "tz": []}

# Create a function to handle MQTT message reception
def on_message(client, userdata, message):
    payload = message.payload.decode()
    fx, fy, fz, tx, ty, tz = map(float, payload.split(","))
    data["time"].append(len(data["time"]))  # Use the time step as the x-coordinate
    data["fx"].append(fx)
    data["fy"].append(fy)
    data["fz"].append(fz)
    data["tx"].append(tx)
    data["ty"].append(ty)
    data["tz"].append(tz)

# Create MQTT client
mqtt_force_client = mqtt.Client()
mqtt_force_client.on_message = on_message
mqtt_force_client.connect(mqtt_broker_host)
mqtt_force_client.subscribe(mqtt_force_topic)
mqtt_force_client.loop_start()

try:
    while True:
        plt.clf()  # Clear the previous plot

        # Plot the force and torque data
        plt.subplot(211)
        plt.plot(data["time"], data["fx"], label='Fx')
        plt.plot(data["time"], data["fy"], label='Fy')
        plt.plot(data["time"], data["fz"], label='Fz')
        
        plt.xlabel('Time Step')
        plt.ylabel('Force')
        plt.title('Real-time Force Plot')
        plt.legend()

        plt.subplot(212)
        plt.plot(data["time"], data["tx"], label='Tx')
        plt.plot(data["time"], data["ty"], label='Ty')
        plt.plot(data["time"], data["tz"], label='Tz')

        plt.xlabel('Time Step')
        plt.ylabel('Torque')
        plt.title('Real-time Torque Plot')
        plt.legend()

        plt.pause(0.01)

except KeyboardInterrupt:
    mqtt_force_client.disconnect()

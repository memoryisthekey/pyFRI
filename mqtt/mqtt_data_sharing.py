import abc
from paho.mqtt import client as mqtt_client


class MqttPub(abc.ABC):
    def __init__(self,  broker="0.0.0.0", port=1883, 
                topic="default/topic", client_id ="id_pub"):
        
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client_id = client_id

    def on_connect(client, userdata, flags, rc);
        if rc == 0:
                print("Connected to MQTT Broker!")
         else:
                print("Failed to connect, return code %d\n", rc)

        self.client = mqtt_client.Client(self.client_id)
        # client.username_pw_set(username, password)
        self.client.on_connect = on_connect
        self.client.connect(self.broker, self.port)
    
    @abc.abstractmethod
    def publish_data(self, data):
        pass
    
class MqttSub(abc.ABC):
    def __init__(self,  broker="0.0.0.0", port=1883, 
                topic="default/topic", client_id ="id_sub"):
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client_id = client_id
        self.subscribe()

    @abc.abstractmethod
    def on_message(self, client, userdata, message):
        pass
    
    def subscribe(self):
        # Create MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_message = on_message
        self.mqtt_client.connect(self.broker)
        self.mqtt_client.subscribe(self.topic)
        self.mqtt_client.loop_start()


# Publishing class overrite
class MqttPublisher(MqttPub):
    def __init__(self):
        super.__init__(self)

    def publish(self, data):
        msg = f"{data[0]},{data[1]},{data[2]}"
        result = self.client.publish(self.topic, payload=msg, qos=0, retain=False)
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{self.topic}`")
        else:
            print(f"Failed to send message to topic {self.topic}")
    
    def on_message(self):
        pass

class MqttReceiver(MqttSub):
    def __init__(self):
        super.__init__(self)
    
        
    # Callback function to handle MQTT message reception
    def on_message(self, client, userdata, message):
        global orientation_data
        payload = message.payload.decode()
        # Assuming payload is in the format "u,v,w" where u, v, and w are orientation values
        u, v, w = map(float, payload.split(","))
        #orientation_data["u"] = u
        #orientation_data["v"] = v
        #orientation_data["w"] = w
        return u, v, w

    


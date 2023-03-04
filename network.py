import paho.mqtt.client as paho
from paho.mqtt import client as mqtt_client
import random
from const import to_place
from main import perform

broker="10.156.248.70"
port=1883
topic = "dropDown"
subscribe_topic="ddu"
client_id = f'python-mqtt-{random.randint(0, 100)}'
client1= paho.Client("control1")                           #create client object


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)

    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    # Sample Payload : operation,marker,Time (If operation is pour)
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic\n")
        str=msg.payload.decode()
        x=str.split(",")
        print(type(x))
        operation=x[0]
        target=x[1]
        if(operation!="pour"):
        timer=None
        else:
        timer=x[2]
        flag=0
        perform_exp(x)
    client.subscribe(subscribe_topic)
    client.on_message = on_message

def publish(data):
    ret= client1.publish("dropDown",str(data))

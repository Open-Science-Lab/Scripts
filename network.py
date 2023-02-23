import paho.mqtt.client as paho
from paho.mqtt import client as mqtt_client
import random

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
    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic\n")
        str=msg.payload.decode()
        x=str.split(",")
        print(type(x))
        to_pick=x[1]
        to_place=x[0]
        to_place_id=0
        option=x[2]
        print(str)
         
        flag=0

        if to_place=='1':
            to_place_id=2
        if to_place=='2':
            to_place_id=34
    client.subscribe(subscribe_topic)
    client.on_message = on_message

def publish(data):
    ret= client1.publish("dropDown",str(data))
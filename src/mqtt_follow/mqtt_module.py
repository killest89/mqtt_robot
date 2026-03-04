import paho.mqtt.client as mqtt
import threading

class MqttClientWrapper(object):
    def __init__(self, broker, port=1883, client_id="", username=None, password=None):
        self.client = mqtt.Client(client_id)
        self.broker = broker
        self.port = port
        self.username = username
        self.password = password
        if self.username is not None and self.password is not None:
            self.client.username_pw_set(self.username, self.password)
        self._is_connected = False

    def connect(self):
        if not self._is_connected:
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()
            self._is_connected = True

    def publish(self, topic, message, qos=1):
        self.connect()
        self.client.publish(topic, message, qos=qos)

    def subscribe(self, topic, callback, qos=1, run_in_thread=False):
        def listen():
            self.connect()
            def on_message(client, userdata, msg):
                callback(msg.topic, msg.payload)
            self.client.subscribe(topic, qos=qos)
            self.client.on_message = on_message
            try:
                while True:
                    pass
            except KeyboardInterrupt:
                self.disconnect()
        if run_in_thread:
            t = threading.Thread(target=listen)
            t.daemon = True
            t.start()
            return t
        else:
            listen()

    def disconnect(self):
        if self._is_connected:
            self.client.loop_stop()
            self.client.disconnect()
            self._is_connected = False

    def publish_and_wait_for_message(self, pub_topic, pub_message, sub_topic, callback, qos=1, timeout=10):
        event = threading.Event()
        def on_message(client, userdata, msg):
            callback(msg.topic, msg.payload)
            event.set()
        self.connect()
        self.client.subscribe(sub_topic, qos=qos)
        self.client.on_message = on_message
        self.client.publish(pub_topic, pub_message, qos=qos)
        event.wait(timeout)
        self.disconnect()

    @staticmethod
    def publish_message(broker, port, topic, message, client_id="", username=None, password=None, qos=1):
        client = mqtt.Client(client_id)
        if username is not None and password is not None:
            client.username_pw_set(username, password)
        client.connect(broker, port, 60)
        client.loop_start()
        client.publish(topic, message, qos=qos)
        client.loop_stop()
        client.disconnect()

    @staticmethod
    def listen_message(broker, port, topic, callback, client_id="", username=None, password=None, qos=1, timeout=None, run_in_thread=False):
        def listen():
            client = mqtt.Client(client_id)
            if username is not None and password is not None:
                client.username_pw_set(username, password)
            event = threading.Event()
            def on_message(client, userdata, msg):
                callback(msg.topic, msg.payload)
                if timeout is not None:
                    event.set()
            client.on_message = on_message
            client.connect(broker, port, 60)
            client.subscribe(topic, qos=qos)
            client.loop_start()
            if timeout is not None:
                event.wait(timeout)
                client.loop_stop()
                client.disconnect()
            else:
                try:
                    while True:
                        pass
                except KeyboardInterrupt:
                    client.loop_stop()
                    client.disconnect()
        if run_in_thread:
            t = threading.Thread(target=listen)
            t.daemon = True
            t.start()
            return t
        else:
            listen()
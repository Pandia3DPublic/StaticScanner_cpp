import datetime
import random
import time

import faker.generator
from faker import Faker
import requests

fake = Faker()

BASE = "http://127.0.0.1:80/api/v0/"

EVERYTHING = True


def randcol():
    x = "#"
    x += "%06x" % random.randint(0, 0xFFFFFF)
    return x


def apiTest(EVERYTHING):
    if EVERYTHING:

        names = ["Base (MTW-Ortung)", "Hand Tracker 1", "Hand Tracker 2", "Hand Tracker 3", "Hand Tracker 4", "Mavic 2"]
        icons = ["fas fa-car", "fas fa-mobile-alt", "fas fa-mobile-alt", "fas fa-mobile-alt", "fas fa-mobile-alt",
                 "fas fa-helicopter"]
        types = ["Vehicle", "Handtracker", "Handtracker", "Handtracker", "Handtracker", "Drone"]
        for x in range(len(names)):
            data = [{"name": names[x], "iconString": icons[x], "colorString": str(randcol()), "type": types[x]}]
            response = requests.post(BASE + "tracker", data=data[0])
            print(response.json())


    posx = 47.725155
    posy = 10.306963

    for x in range(100):
        posx += random.uniform(-0.0001, 0.0001)
        posy += random.uniform(-0.0001, 0.0001)
        data = [{"trackerID": random.randint(5, 10), "lat": posx, "lon": posy}]
        response = requests.post(BASE + "ping", data=data[0])
        print(response.json())
        time.sleep(0.5)

    one_minute = datetime.timedelta(minutes=1)
    data = [{"from": (datetime.datetime.now() - one_minute).timestamp()}]
    print(data)
    response = requests.get(BASE + "ping", data=data[0])
    print(response.json())
    response = requests.get(BASE + "currentpings")
    print(response.json())


if __name__ == "__main__":
    apiTest(True)

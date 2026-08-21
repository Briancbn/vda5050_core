"""Run a VDA5050 master that observes the paired robot adapter."""

import logging
import os
import time
# import uvicorn
# from  fastapi import FastAPI
# import dotenv
# from pydantic import BaseField, Field
from vda5050_core.types import InstantActions, OperatingMode
from vda5050_core.master import VDA5050Master
from vda5050_core.transport import create_mqtt_client
from uuid import uuid4

from datetime import datetime, timezone

BROKER_URI = os.environ.get("MQTT_BROKER", "tcp://localhost:1883")
MQTT_CLIENT_ID = os.environ.get("MASTER_MQTT_CLIENT_ID", "example-master")
MANUFACTURER = os.environ.get("VDA5050_MANUFACTURER", "Manufacturer")
SERIAL_NUMBER = os.environ.get("VDA5050_SERIAL_NUMBER", "S001")
LOGGER = logging.getLogger(__name__)

IS_READY = False

class MasterObserver:
    def on_connect(self, agv_id) -> None:
        LOGGER.info("AGV connected: %s", agv_id)

    def on_offline(self, agv_id) -> None:
        LOGGER.info("AGV offline: %s", agv_id)

    def on_connection_broken(self, agv_id) -> None:
        LOGGER.warning("AGV connection broken: %s", agv_id)

    def on_state(self, agv_id, state) -> None:
        position = state.agv_position
        pose = None if position is None else (position.x, position.y, position.theta)
        LOGGER.info(
            "State: %s",
            state.json(),
        )

        if state.operating_mode == OperatingMode.AUTOMATIC:
            print("yay")
            global IS_READY
            IS_READY = True


def main() -> None:
    # app = FastAPI()
    # @app.get("/")
    # async def root():
    #     return {"message": "Hello World"}

    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s"
    )
    mqtt_client = create_mqtt_client(BROKER_URI, MQTT_CLIENT_ID)
    master = VDA5050Master.make(mqtt_client)
    master_observer = MasterObserver()

    master.on_connect(master_observer.on_connect)
    master.on_offline(master_observer.on_offline)
    master.on_state(master_observer.on_state)
    master.on_connection_broken(master_observer.on_connection_broken)

    master.connect()
    master.onboard_agv(MANUFACTURER, SERIAL_NUMBER)
    LOGGER.info(
        "Master listening for %s/%s via %s",
        MANUFACTURER,
        SERIAL_NUMBER,
        BROKER_URI,
    )

    utc_string = datetime.now(timezone.utc).isoformat()
    actions = InstantActions.from_json(
        {
            "headerId": 1,
            "timestamp": utc_string,
            "version": "2.0.0",
            "manufacturer": MANUFACTURER,
            "serialNumber": SERIAL_NUMBER,
            "actions": [
                {
                    "actionType": "pick_all",
                    "actionId": str(uuid4()),
                    "blockingType": "HARD"
                }
            ]
        }
    )

    try:
        while not IS_READY:
            time.sleep(1)

        result = master.assign_instant_actions(
            MANUFACTURER,
            SERIAL_NUMBER,
            actions
        )
        print(result.decision)
        print([error.json() for error in result.errors])

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        master.offboard_agv(MANUFACTURER, SERIAL_NUMBER)
        master.disconnect()


if __name__ == "__main__":

    main()

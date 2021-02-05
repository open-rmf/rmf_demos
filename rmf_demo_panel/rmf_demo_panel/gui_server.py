
# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from flask import Flask, render_template, jsonify
import os
import sys
import requests
import json

###############################################################################

app = Flask(__name__, static_url_path="/static")
dashboard_config = {"world_name": ""}


@app.route("/")
def home():
    return render_template("index.html")


@app.route("/dashboard_config", methods=['GET'])
def config():
    config = jsonify(dashboard_config)
    return config


###############################################################################

def main(args=None):
    server_ip = "0.0.0.0"
    port_num = 5000

    if "WEB_SERVER_IP_ADDRESS" in os.environ:
        server_ip = os.environ['WEB_SERVER_IP_ADDRESS']
        print(f"Set Server IP to: {server_ip}:{port_num}")

    if "DASHBOARD_CONFIG_PATH" in os.environ:
        config_path = os.environ['DASHBOARD_CONFIG_PATH']

        if not config_path:
            print(f"WARN! env DASHBOARD_CONFIG_PATH is empty...")
        elif not os.path.exists(config_path):
            print(f"File [{config_path}] doesnt exist")
            raise FileNotFoundError
        else:
            try:
                f = open(config_path, 'r')
                global dashboard_config
                dashboard_config = json.load(f)
            except Exception as err:
                print(f"Failed to read [{config_path}] dashboard config file")
                raise err
    else:
        print(f"WARN! env DASHBOARD_CONFIG_PATH is not specified...")

    print("Starting Dispatcher Dashboard GUI Server")
    app.run(host=server_ip, debug=False, port=port_num)


if __name__ == "__main__":
    main(sys.argv)

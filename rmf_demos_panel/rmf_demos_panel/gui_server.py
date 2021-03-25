
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
dashboard_config = {"world_name": "EMPTY_DASHBOARD_CONFIG",
                    "task": {"Delivery": {}, "Loop": {}, "Clean": {}}}


# Download bundle from gh page, This will download the webpack bundle if the
# bundle is not available locally, thus internet is required during download
def download_webpack():
    script_dir = os.path.dirname(os.path.realpath(__file__))
    bundle_path = f"{script_dir}/static/dist/app.bundle.js"

    # source_url is deployed during a push sequence within git page action,
    # it is advice to update the version (in '.github/workflows/gh_page.yml')
    # whenever a new feature is introduced
    source_url = "https://open-rmf.github.io/rmf_demos/v1.0.0/app.bundle.js"
    print("Check path ", bundle_path)
    if os.path.isfile(bundle_path):
        print("BundleJS File exists, SKIP!")
    else:
        print(f"BundleJS File does not exist, Download it from {source_url}")
        response = requests.get(source_url)
        if response.status_code == 200:
            dir_path = os.path.dirname(bundle_path)
            if not os.path.isdir(dir_path):
                print(f"Dir path: {dir_path} doesnt exist, create dir")
                os.mkdir(dir_path)
            open(bundle_path, 'wb').write(response.content)
            print(f"Bundle Download completed: {bundle_path}")
        else:
            raise UserWarning(f"Failed to download bundle from: {source_url}")

###############################################################################


@app.route("/")
def home():
    return render_template("index.html")


# Dashboard Config for each "World"
@app.route("/dashboard_config", methods=['GET'])
def config():
    config = jsonify(dashboard_config)
    return config


###############################################################################

def main(args=None):
    server_ip = "0.0.0.0"
    port_num = 5000

    download_webpack()

    if "WEB_SERVER_IP_ADDRESS" in os.environ:
        server_ip = os.environ['WEB_SERVER_IP_ADDRESS']
        print(f"Set Server IP to: {server_ip}:{port_num}")

    if "DASHBOARD_CONFIG_PATH" in os.environ:
        config_path = os.environ['DASHBOARD_CONFIG_PATH']

        if not config_path:
            print(f"WARN! env DASHBOARD_CONFIG_PATH is empty...")
        elif not os.path.exists(config_path):
            raise FileNotFoundError(f"\n File [{config_path}] doesnt exist")
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

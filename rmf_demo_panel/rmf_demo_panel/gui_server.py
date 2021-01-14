
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


from flask import Flask, render_template
import os
import sys
import requests

app = Flask(__name__, static_url_path="/static")


@app.route("/")
def home():
    return render_template("index.html")


def main(args=None):
    server_ip = "0.0.0.0"
    port_num = 5000

    if "WEB_SERVER_IP_ADDRESS" in os.environ:
        server_ip = os.environ['WEB_SERVER_IP_ADDRESS']
        print(f"Set Server IP to: {server_ip}:{port_num}")

    print("Starting Dispatcher Dashboard GUI Server")
    app.run(host=server_ip, debug=False, port=port_num)


if __name__ == "__main__":
    main(sys.argv)

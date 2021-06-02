import glob
import os
import json
from pathlib import Path

# Get the directories names inside the folder rmf_dashboard_resources
worlds = next(os.walk('.'))[1]

for world in worlds:
    app_config = {
        "dispensers": {},
        "robots": {},
        "logos": {}
    }
    for topic in app_config.keys():
        # Get all JSON files inside the folder of each world
        files = Path(world + '/' + topic + '/').glob("**/*.json")
        for file in files:
            with open(file) as json_file:
                app_config[topic].update(json.load(json_file))

    with open(world + '/main.json', 'w') as main_file:
        main_file.write(json.dumps(app_config))

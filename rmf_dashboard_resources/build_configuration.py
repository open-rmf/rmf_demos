import glob, os
import json
from pathlib import Path

maps = ['office', 'airport_terminal']

for map in maps:
    app_config = {
        "dispensers":{},
        "robots":{}
    }
    for topic in app_config.keys():    
        # Get all JSON files inside the folder of each MAP
        files = Path(map + '/' + topic + '/').glob("**/*.json")
        for file in files:
            with open(file) as json_file:
                app_config[topic].update(json.load(json_file)) 
    
    with open(map + '/main.json', 'w') as main_file:
        main_file.write(json.dumps({map:app_config}))

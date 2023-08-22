import yaml



config = {
    'database': {
        - [[1.00, 0.37],
           [1.00, 1.30]]
    }
}

with open('config.yaml','w') as f:
    yaml.dump(config, f)
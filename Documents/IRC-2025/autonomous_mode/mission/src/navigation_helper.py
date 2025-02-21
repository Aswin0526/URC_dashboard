import yaml

with open('config.yaml', 'r') as config:
    data = yaml.full_load(config)

PROXIMITY_LIMIT = data.get("PROXIMITY_LIMIT")
LEFT_X = data.get("LEFT_X")
RIGHT_X = data.get("RIGHT_X")

def is_boxnear(box, depth):
    x, y, _, _ = map(int, box.xywh[0])
    # print("my test" + str(depth.shape)) # (720,1280)
    print("depth", depth[y, x])
    if depth[y, x] <= PROXIMITY_LIMIT:
        return True
    else:
        return False

def adjust_direction(box):
    x, _, _, _ = box.xywh[0]
    if x < LEFT_X:
        return "left"
    if x > RIGHT_X:
        return "right"
    return None

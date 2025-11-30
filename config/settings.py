import os

# ========== Path Config ==========
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SAVE_PATH = os.path.join(ROOT_DIR, "media")

# ========== Drone Config ==========
DEFAULT_Z = -2.0
MOVE_VELOCITY = 1.0
TARGET_DISTANCE_THRESHOLD = 5.0  # meters
CHANGE_DIRECTION_DISTANCE_THRESHOLD = 10000  # mm

# ========== Image Config ==========
VIDEO_FPS = 30
# SUBWIN_WIDTH = 640
# SUBWIN_HEIGHT = 360
SUBWIN_WIDTH = 400
SUBWIN_HEIGHT = 225
SMOOTHING_FACTOR = 0.1

# ========== Road Network Coordinates ==========
ROAD_COORDINATES = {
    "A": (129.2, -225.8), "B": (129.2, -160.0), "C": (129.3, -89.8), "D": (128.2, -9.6),
    "E": (133.1, 72.2), "F": (139.6, 160.6), "G": (3.6, -229.5), "H": (3.6, -157.9),
    "I": (4.8, -92.4), "J": (6.0, -9.6), "K": (5.0, 72.2), "L": (0.4, 160.6),
    "M": (-125.0, -229.5), "N": (-125.4, -156.1), "O": (-125.4, -88.3), "P": (-125.4, -7.6),
    "Q": (-123.4, 72.2), "R": (-130.8, 160.6), "S": (-252.7, -229.5), "T": (-253.3, -162.3),
    "U": (-253.3, -93.2), "V": (-253.3, -11.4), "W": (-253.3, 72.6), "X": (-253.3, 159.4)
}

ROAD_CONNECTIONS = [
    ("A","B"),("A","G"),("B","C"),("B","H"),("C","D"),("C","I"),("D","E"),("D","J"),
    ("E","F"),("E","K"),("F","L"),("G","H"),("G","M"),("H","I"),("H","N"),("I","J"),
    ("I","O"),("J","K"),("J","P"),("K","L"),("K","Q"),("L","R"),("M","N"),("M","S"),
    ("N","O"),("N","T"),("O","P"),("O","U"),("P","Q"),("P","V"),("Q","R"),("Q","W"),
    ("R","X"),("S","T"),("T","U"),("U","V"),("V","W"),("W","X")
]
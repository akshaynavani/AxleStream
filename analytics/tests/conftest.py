import os
import sys

# Make `axle_analytics` importable when running `pytest analytics/tests` from the repo root.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

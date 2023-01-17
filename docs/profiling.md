# Profiling

Currently to profile, we use cProfile and Snakeviz to visualize it. Snakeviz is included in the Pipfile dev packages.
In the future the project will be configured to support profiling for multi-threaded applications.

Run as follows:
python -m cProfile -o <output>.prof run.py <usual params>
snakeviz <output>.prof

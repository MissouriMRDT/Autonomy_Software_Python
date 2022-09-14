/* 
 * Mars Rover Design Team
 * profiling.md
 * 
 * Created on Jan 18, 2021
 * Updated on Aug 21, 2022
 */

# Profiling

Currently, to profile, we use cProfile and Snakeviz to visualize it. Snakeviz is included in the Pipfile dev packages.
In the future the project will be configured to support profiling for multithreading applications.

Run as follows:
python -m cProfile -o <output>.prof run.py <usual params>
snakeviz <output>.prof

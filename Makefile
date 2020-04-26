FUNCTION=depthFirstSearch

dfs:
	 python3 pacman.py -l tinyMaze -p SearchAgent -a fn=$(FUNCTION)

ucs:
	python3 pacman.py -l mediumMaze -p SearchAgent -a fn=ucs

problem4:
	python3 pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

debug:
	 pudb3 pacman.py -l tinyMaze -p SearchAgent -a fn=$(FUNCTION)

test:
	python3 autograder.py

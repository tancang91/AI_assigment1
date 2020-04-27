FUNCTION=depthFirstSearch

dfs:
	 python3 pacman.py -l tinyMaze -p SearchAgent -a fn=$(FUNCTION)

ucs:
	python3 pacman.py -l mediumMaze -p SearchAgent -a fn=ucs

problem4:
	python3 pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic

problem5:
	python3 pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem

debug:
	 pudb3 pacman.py -l tinyMaze -p SearchAgent -a fn=$(FUNCTION)

test:
	python3 autograder.py

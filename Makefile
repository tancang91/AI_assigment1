FUNCTION=depthFirstSearch

run:
	 python3 pacman.py -l tinyMaze -p SearchAgent -a fn=$(FUNCTION)

debug:
	 pudb3 pacman.py -l tinyMaze -p SearchAgent -a fn=$(FUNCTION)

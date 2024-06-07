exec:
	g++ --std=c++17 -o bin/forest.x src/forest.cpp

compute:
	./bin/forest.x > ./data/forest.csv

plot:
	python3 plot_data.py

clean:
	rm bin/* data/* images/*
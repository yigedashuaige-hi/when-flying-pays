.PHONY: reproduce statistics figures clean

reproduce: statistics figures

statistics:
	python3 analysis/reproduce_statistics.py

figures:
	MPLCONFIGDIR=/tmp/mplconfig_when_flying_pays python3 analysis/generate_figures.py

clean:
	rm -rf reproduced

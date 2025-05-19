all: grenchen_20250514

grenchen_20250514:
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_1.tst output/result_20250514_grenchen_1.txt
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_2.tst output/result_20250514_grenchen_2.txt
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_3.tst output/result_20250514_grenchen_3.txt
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_4.tst output/result_20250514_grenchen_4.txt
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_5.tst output/result_20250514_grenchen_5.txt
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_6.tst output/result_20250514_grenchen_6.txt
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_7.tst output/result_20250514_grenchen_7.txt
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_8.tst output/result_20250514_grenchen_8.txt
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_9.tst output/result_20250514_grenchen_9.txt


all: birrfeld_20250523 grenchen_20250514 birrfeld_20250110

birrfeld_20250523:
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_1.tst output/result_20250523_birrfeld_1.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_2.tst output/result_20250523_birrfeld_2.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_3.tst output/result_20250523_birrfeld_3.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_4.tst output/result_20250523_birrfeld_4.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_5.tst output/result_20250523_birrfeld_5.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_6.tst output/result_20250523_birrfeld_6.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_7.tst output/result_20250523_birrfeld_7.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_8.tst output/result_20250523_birrfeld_8.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_9.tst output/result_20250523_birrfeld_9.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_10.tst output/result_20250523_birrfeld_10.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_11.tst output/result_20250523_birrfeld_11.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_12.tst output/result_20250523_birrfeld_12.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_13.tst output/result_20250523_birrfeld_13.txt
	./src/sim/sim --autorun testflights/20250523_birrfeld/test_birrfeld_14.tst output/result_20250523_birrfeld_14.txt

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

birrfeld_20250110:
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_1_1.tst output/result_20250110_birrfeld_1_1.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_1_2.tst output/result_20250110_birrfeld_1_2.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_1.tst output/result_20250110_birrfeld_2_1.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_2.tst output/result_20250110_birrfeld_2_2.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_3.tst output/result_20250110_birrfeld_2_3.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_4.tst output/result_20250110_birrfeld_2_4.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_5.tst output/result_20250110_birrfeld_2_5.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_6.tst output/result_20250110_birrfeld_2_6.txt


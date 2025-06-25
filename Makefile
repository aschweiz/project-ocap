all: birrfeld_20250523 grenchen_20250514 birrfeld_20250222 birrfeld_20250110

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

grenchen_20250408:
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_01.tst output/result_20250408_grenchen_01.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_02.tst output/result_20250408_grenchen_02.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_03.tst output/result_20250408_grenchen_03.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_04.tst output/result_20250408_grenchen_04.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_05.tst output/result_20250408_grenchen_05.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_06.tst output/result_20250408_grenchen_06.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_07.tst output/result_20250408_grenchen_07.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_08.tst output/result_20250408_grenchen_08.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_09.tst output/result_20250408_grenchen_09.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_10.tst output/result_20250408_grenchen_10.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_11.tst output/result_20250408_grenchen_11.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_12.tst output/result_20250408_grenchen_12.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_13.tst output/result_20250408_grenchen_13.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_14.tst output/result_20250408_grenchen_14.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_15.tst output/result_20250408_grenchen_15.txt
	./src/sim/sim --autorun testflights/20250408_grenchen/test_grenchen_20250408_16.tst output/result_20250408_grenchen_16.txt

grenchen_20250514:
	./src/sim/sim --autorun testflights/20250514_grenchen/test_grenchen_1.tst output/result_20250514_grenchen_1.txt

birrfeld_20250222:
	./src/sim/sim --autorun testflights/20250222_birrfeld/test_msw_20250222_1.tst output/result_20250222_birrfeld_1.txt
	./src/sim/sim --autorun testflights/20250222_birrfeld/test_msw_20250222_2.tst output/result_20250222_birrfeld_2.txt
	./src/sim/sim --autorun testflights/20250222_birrfeld/test_msw_20250222_3.tst output/result_20250222_birrfeld_3.txt
	./src/sim/sim --autorun testflights/20250222_birrfeld/test_msw_20250222_4.tst output/result_20250222_birrfeld_4.txt
	./src/sim/sim --autorun testflights/20250222_birrfeld/test_msw_20250222_5.tst output/result_20250222_birrfeld_5.txt
	./src/sim/sim --autorun testflights/20250222_birrfeld/test_msw_20250222_6.tst output/result_20250222_birrfeld_6.txt

birrfeld_20250110:
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_1_1.tst output/result_20250110_birrfeld_1_1.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_1_2.tst output/result_20250110_birrfeld_1_2.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_1.tst output/result_20250110_birrfeld_2_1.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_2.tst output/result_20250110_birrfeld_2_2.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_3.tst output/result_20250110_birrfeld_2_3.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_4.tst output/result_20250110_birrfeld_2_4.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_5.tst output/result_20250110_birrfeld_2_5.txt
	./src/sim/sim --autorun testflights/20250110_birrfeld/test_msw_20250110_2_6.tst output/result_20250110_birrfeld_2_6.txt


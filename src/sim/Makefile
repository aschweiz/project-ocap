all: world_coords.h flight_path.h win_base.h win_world.h win_time.h win_objects.h \
ctl_slider.h display_util.h workspace.h radio_link.h ads_l_integration.h lon_lat_util.h \
time_range.h rect.h flight_path_config.h test_set.h predicted_trace.h parse_pflag.h \
 \
../common/linalg.h
	g++ -ggdb -o sim -I../common -I../libocap -I../libadsl \
main.cpp world_coords.cpp workspace.cpp radio_link.cpp ads_l_integration.cpp \
flight_path.cpp flight_path_config.cpp test_set.cpp lon_lat_util.cpp \
rect.cpp time_range.cpp win_base.cpp win_world.cpp win_time.cpp win_objects.cpp \
ctl_slider.cpp display_util.cpp predicted_trace.cpp parse_pflag.c \
../common/linalg.cpp \
../libocap/libocap.a \
../libadsl/libadsl.a \
-lglut -lGLU -lGL -lm

import rosbag

import math
import numpy as np
from bokeh.plotting import ColumnDataSource, figure, output_file, show
from bokeh.layouts import column, row
from bokeh.models import CustomJS, Slider, TapTool  


# read bag, convert polar coordinates to points and save to array
def bag_scan_to_points(bag_file_path, scan_topic, number_lidar_ranges, rotation_angle):
    # open bag file
    bag = rosbag.Bag(bag_file_path)

    # save distances to array
    number_messages = bag.get_message_count(topic_filters=[scan_topic])
    timestamps = []
    distances = np.full((number_messages * number_lidar_ranges, 1), None)

    count = 0
    for topic, msg, time in bag.read_messages(topics=[scan_topic]):
        timestamps.append(time.to_time())
        distances[count*number_lidar_ranges:count*number_lidar_ranges+number_lidar_ranges, 0] = np.array(msg.ranges)
        # print(np.where(np.array(msg.ranges) == 0.0)[0].shape) #numeber of ranges where distance is zero
        count += 1
    bag.close()

    time_difference = timestamps[-1] - timestamps[0]

    if number_lidar_ranges == 360:
        angles = np.arange(0, number_lidar_ranges)
    else:
        angles = np.linspace(0, 360, number_lidar_ranges)
    angles = np.reshape(angles, (number_lidar_ranges, 1))
    # rotation
    # angles = np.roll(angles, rotation_angle)
    angles = np.roll(angles, int(np.rint(rotation_angle/(360/number_lidar_ranges))))

    # distance to points
    points = np.full((distances.shape[0], 2), None)
    index_counter = 0
    for i in range(points.shape[0]):
        if index_counter >= number_lidar_ranges:
            index_counter = 0
        if distances[i, 0] != float("inf") and distances[i, 0] != float("-inf") and distances[i, 0] > 0.0:
            # x
            points[i, 0] = math.cos(math.radians(angles[index_counter, 0])) * distances[i, 0]
            # y
            points[i, 1] = math.sin(math.radians(angles[index_counter, 0])) * distances[i, 0]
        else:
            points[i, :] = [None, None]
        index_counter += 1

    return points, angles, number_messages, time_difference


# fit line to given points in angle intervals
def fit_line(points, angles, angle_zones, axis_focus):
    residuals = []
    poly = []
    if len(angle_zones) != len(axis_focus):
        print("check angle_zones and axis_focus arrays for same length")
    for i in range(len(angle_zones)):

        # get index of points in angle zone
        index = np.array([])
        for j in range(0, len(angle_zones[i]), 2):
            # check if there is angle transition of 360 to 0 degrees
            if angle_zones[i][j] < angle_zones[i][j+1]:
                index = np.append(index, np.where((angle_zones[i][j] <= angles) & (angles <= angle_zones[i][j+1]))[0])
            else:
                index = np.append(index, np.where(angles >= angle_zones[i][j])[0])
                index = np.append(index, np.where(angles <= angle_zones[i][j+1])[0])

        # get points from index
        fit_points = points[index.astype('int')]
        fit_points = fit_points[fit_points!=None].reshape(-1, 2).astype('float64')

        # check for vertical line
        if axis_focus[i] == 'x':
            p, res, _, _, _ = np.polyfit(fit_points[:, 0], fit_points[:, 1], 1, full=True)
        elif axis_focus[i] == 'y':
            p, res, _, _, _ = np.polyfit(fit_points[:, 1], fit_points[:, 0], 1, full=True)

        residuals.append(res / fit_points.shape[0])
        poly.append(p)
    return np.sqrt((sum(residuals)/len(residuals))), poly


# fit line to all points of the entire measurement in angle intervals
def fit_line_all_data(points, number_lidar_ranges, angles, angle_zones, axis_focus):
    number_iterations = int(points.shape[0]/number_lidar_ranges)
    scattering = []
    line = [[0, 0]]*len(angle_zones)
    for i in range(number_iterations):
        start_index = i*number_lidar_ranges
        stop_index = i*number_lidar_ranges + number_lidar_ranges
        scat, poly = fit_line(points[start_index:stop_index, :], angles, angle_zones, axis_focus)
        scattering.append(scat)
        line = [[i1+j1 for i1, j1 in zip(i,j)] for i, j in zip(line, poly)]
    line = [[j/number_iterations for j in i] for i in line]
    # return average scattering and average lines
    return (sum(scattering)/len(scattering)), line


def plot_fitted_line(plot, poly, linspace, axis_focus, color):
    if len(poly) != len(linspace):
        print("check poly and linspace arrays for same length")
    for i in range(len(linspace)):
        if axis_focus[i] == 'x':
            for j in range(0, len(linspace[i]), 2):
                x = np.linspace(linspace[i][j], linspace[i][j+1], 2)
                y = np.polyval(poly[i], x)
                plot.line(x, y, color=color, line_width=2)
        elif axis_focus[i] == 'y':
            for j in range(0, len(linspace[i]), 2):
                y = np.linspace(linspace[i][j], linspace[i][j+1], 2)
                x = np.polyval(poly[i], y)
                plot.line(x, y, color=color, line_width=2)


# load data and prepare for bokeh
#################################

# rplidar
rplidar_number_lidar_ranges = 360
rplidar_points, rplidar_angles, number_messages_rp, time_difference_rp = bag_scan_to_points('bag_files/rplidar_1.bag', '/scan_rplidar', rplidar_number_lidar_ranges, -51)
rplidar_points_bokeh_initial = ColumnDataSource(data=dict(x=rplidar_points[:rplidar_number_lidar_ranges, 0], y=rplidar_points[:rplidar_number_lidar_ranges, 1], angle=rplidar_angles))
rplidar_points_bokeh = ColumnDataSource(data=dict(x=rplidar_points[:, 0], y=rplidar_points[:, 1]))

# ydlidar
ydlidar_number_lidar_ranges = 2019
ydlidar_points, ydlidar_angles, number_messages_yd, time_difference_yd= bag_scan_to_points('bag_files/ydlidar_1.bag', '/scan_ydlidar', ydlidar_number_lidar_ranges, 72)
ydlidar_points_bokeh_initial = ColumnDataSource(data=dict(x=ydlidar_points[:ydlidar_number_lidar_ranges, 0], y=ydlidar_points[:ydlidar_number_lidar_ranges, 1], angle=ydlidar_angles))
ydlidar_points_bokeh = ColumnDataSource(data=dict(x=ydlidar_points[:, 0], y=ydlidar_points[:, 1]))


# bokeh configuration
#####################

TOOLTIPS = [
    ("index", "$index"),
    ("name", "$name"),
    ("(x,y)", "($x, $y)"),
    ("angle", "@angle"),
]

p = figure(plot_width=900, plot_height=720, x_range=(-4, 3.5), y_range=(-3, 3), tooltips=TOOLTIPS)

slider_time = Slider(start=0, end=119, value=0, step=0.05, title="time [sec]",)

# update lidar points by changing slider
callback = CustomJS(args=dict(rp_source=rplidar_points_bokeh_initial, rp_allpoints=rplidar_points_bokeh, yd_source=ydlidar_points_bokeh_initial, yd_allpoints=ydlidar_points_bokeh, number_messages_rp=number_messages_rp, number_messages_yd=number_messages_yd,time_difference_rp=time_difference_rp , time_difference_yd=time_difference_yd, time=slider_time),
                    code="""
    const rp_data = rp_source.data;
    const yd_data = yd_source.data;
    const rp_points = rp_allpoints.data;
    const yd_points = yd_allpoints.data;
    const rp_t =  Math.round((number_messages_rp/time_difference_rp)*time.value);
    const yd_t =  Math.round((number_messages_yd/time_difference_yd)*time.value);

    for (var i = 0; i < 360; i++) {
        rp_data['x'][i] = rp_points['x'][rp_t*360 + i]
        rp_data['y'][i] = rp_points['y'][rp_t*360 + i]
    }
    for (var i = 0; i < 2019; i++) {
        yd_data['x'][i] = yd_points['x'][yd_t*2019 + i]
        yd_data['y'][i] = yd_points['y'][yd_t*2019 + i]
    }
    rp_source.change.emit();
    yd_source.change.emit();
""")

slider_time.js_on_change('value', callback)

# log distance between two points in console
selected_point = []
callback_distance = CustomJS(args=dict(rp_source=rplidar_points_bokeh_initial, yd_source=ydlidar_points_bokeh_initial, selected_point=selected_point), code='''
    var selected_index_rp = rp_source.selected.indices;
    var selected_index_yd = yd_source.selected.indices;

    // first point selected
    if (selected_point.length == 0){

        // selected point is rplidar
        if (selected_index_rp.length > 0 && selected_index_yd == 0) {
            var point_x = rp_source.data.x[selected_index_rp]
            var point_y = rp_source.data.y[selected_index_rp]
            selected_point.push(point_x);
            selected_point.push(point_y);
            console.log("first rp point selected is " + point_x + " " + point_y)
        }

        // selected point is rplidar
        else if (selected_index_yd.length > 0 && selected_index_rp == 0) {
            var point_x = yd_source.data.x[selected_index_yd]
            var point_y = yd_source.data.y[selected_index_yd]
            selected_point.push(point_x);
            selected_point.push(point_y);
            console.log("first yd point selected is " + point_x + " " + point_y)
        }

        else {
            console.log("click on free space before selecting first point and select unique point!")
        }
    }

    // second point selected
    else{
        // selected point is rplidar
        if (selected_index_rp.length > 0 && selected_index_yd == 0) {
            var point_x = rp_source.data.x[selected_index_rp]
            var point_y = rp_source.data.y[selected_index_rp]
            console.log("second rp point selected is " + point_x + " " + point_y)
            var distance = Math.sqrt( Math.pow(Math.abs(point_x - selected_point[0]), 2) + Math.pow(Math.abs(point_y - selected_point[1]), 2) )
            console.log("distance: " + distance)
            selected_point.pop();
            selected_point.pop();
        }

        // selected point is rplidar
        else if (selected_index_yd.length > 0 && selected_index_rp == 0) {
            var point_x = yd_source.data.x[selected_index_yd]
            var point_y = yd_source.data.y[selected_index_yd]
            console.log("second yd point selected is " + point_x + " " + point_y)
            var distance = Math.sqrt( Math.pow(Math.abs(point_x - selected_point[0]), 2) + Math.pow(Math.abs(point_y - selected_point[1]), 2) )
            console.log("distance: " + distance)
            selected_point.pop();
            selected_point.pop();
        }

        else {
            console.log("click on free space before selecting first point and select unique point!")
        }
    }
    //const x = cb_data.geometries.x;
    //const y = cb_data.geometries.y;
    //console.log("Tap at Screen points X: " + x + " Y: " + y); 
    ''')

p.add_tools(TapTool(callback=callback_distance))


layout = column(
    p,
    slider_time,
)

output_file("lidar_comparison.html")


# plot data
###########
rplidar_color = 'orangered'
ydlidar_color = 'deepskyblue'

# fit and plot lines in defined angle areas
angle_zones = [[147, 157, 159, 187], [187, 190], [191, 245], [246, 277], [277, 318], [337, 21], [23, 26, 29, 40], [43, 65, 76, 117, 122, 131, 137, 144]]
linspace = [[2.308, 1.451, 1.32, -0.441], [-3.44, -2.23], [-2.22, -0.96], [-0.925, 0.318], [0.344, 2.67], [-1.031, 0.9], [1.172, 1.338, 1.522, 2.316], [2.592, 1.089, -1.225, 0.5, -1.451, -2.022, -2.51, -3.302]]
axis_focus = ['y', 'x', 'x', 'x', 'x', 'y',  'y', 'x']

# rp_scattering, rp_p = fit_line(rplidar_points[:rplidar_number_lidar_ranges, :], rplidar_angles, angle_zones, axis_focus)
# print("scattering RPLIDAR: " + str(rp_scattering[0]) + " m")
# plot_fitted_line(p, rp_p, linspace, axis_focus, rplidar_color)
rp_scattering_all, rp_line_all= fit_line_all_data(rplidar_points, rplidar_number_lidar_ranges, rplidar_angles, angle_zones, axis_focus)
print("scattering RPLIDAR: " + str(rp_scattering_all[0]) + " m")
plot_fitted_line(p, rp_line_all, linspace, axis_focus, rplidar_color)

# yd_scattering, yd_p = fit_line(ydlidar_points[:ydlidar_number_lidar_ranges, :], ydlidar_angles, angle_zones, axis_focus)
# print("scattering YDLIDAR: " + str(yd_scattering[0]) + " m")
# plot_fitted_line(p, yd_p, linspace, axis_focus, ydlidar_color)
yd_scattering_all, yd_line_all= fit_line_all_data(ydlidar_points, ydlidar_number_lidar_ranges, ydlidar_angles, angle_zones, axis_focus)
print("scattering YDLIADR: " + str(yd_scattering_all[0]) + " m")
plot_fitted_line(p, yd_line_all, linspace, axis_focus, ydlidar_color)

# plot lidar points
p.circle(x=0, y=0, color='black', name="lidar position", size=10, alpha=0.5)
p.circle('x', 'y', source=rplidar_points_bokeh_initial, color=rplidar_color, legend_label="RPLIDAR A2 (rays: "+ str(rplidar_number_lidar_ranges) +", scattering: "+str("%.4f" % rp_scattering_all[0])+" m)", name="rplidar", size=2, alpha=0.5)
p.circle('x', 'y', source=ydlidar_points_bokeh_initial, color=ydlidar_color, legend_label="YDLIDAR TG15 (rays: "+ str(ydlidar_number_lidar_ranges) +", scattering: "+str("%.4f" % yd_scattering_all[0])+" m)", name="ydlidar", size=2, alpha=0.5)

show(layout)


# calculate distances
#####################

# measure distance between lines and return difference to reference distance
def offset_lidar_reference_measurement(poly_rp, poly_yd, line_index, position, true_distance):
    distance_rp = abs(np.polyval(poly_rp[line_index[0]], [position]) - np.polyval(poly_rp[line_index[1]], [position]))
    distance_yd = abs(np.polyval(poly_yd[line_index[0]], [position]) - np.polyval(poly_yd[line_index[1]], [position]))

    error_rp = distance_rp - true_distance
    error_yd = distance_yd - true_distance
    
    # print(distance_rp[0])
    # print(error_rp[0])
    # print(distance_yd[0])
    # print(error_yd[0])

    return error_rp[0], error_yd[0]


line_index = []
position = []
true_value = []

# 1. unter Tischen, Wand - Wand, 0.15 m zur Wand
line_index.append([0, 6])
position.append(2.17)
true_value.append(6.155)

# 2. unter Tischen, Wand - Wand. 0.55 m zur Wand
line_index.append([0, 6])
position.append(1.77)
true_value.append(6.150)

# 3. vor Tischen, Wand - Wand, 1.3 m zur Wand
line_index.append([0, 6])
position.append(1.02)
true_value.append(6.147)

# 4. Wand - Schrank, 1.5 m zur Wand
line_index.append([0, 5])
position.append(0.823)
true_value.append(5.709)

# 5. Wand - Schrank, 2.3 m zur Wand
line_index.append([0, 5])
position.append(0.023)
true_value.append(5.709)

# 6. Quer, Sideboard - Wand, 0.6 m zur Wand
line_index.append([4, 7])
position.append(2.118)
true_value.append(4.77)

total_error_rp = []
total_error_yd = []
for i in range(len(line_index)):
    error_rp, error_yd = offset_lidar_reference_measurement(rp_line_all, yd_line_all, line_index[i], position[i], true_value[i])
    total_error_rp.append(error_rp)
    total_error_yd.append(error_yd)

print("error RPLIDAR: " + str(np.mean(total_error_rp)) + " m")
print("error YDLIDAR: " + str(np.mean(total_error_yd)) + " m")

# print("variance RPLIDAR: " + str(np.var(total_error_rp)) + " m")
# print("variance YDLIDAR: " + str(np.var(total_error_yd)) + " m")
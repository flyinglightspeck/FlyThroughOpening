
from util import *

R=1

markers = ['o', 's', '^', 'D', 'h', 'v']
colors = ["#0072BD", "#8B6C5C", "#7E2F8E", "#edb120", "#000000", "#4DBEEE"]

# Improvement of time
x = [i for i in range(1, 11)]
y_lists = [
[0.80,0.69,0.59,0.50,0.43,0.37,0.31,0.25,0.18,0.12],
[0.65,0.58,0.50,0.44,0.38,0.33,0.27,0.21,0.15,0.08],
[0.52,0.44,0.39,0.34,0.29,0.25,0.19,0.14,0.08,0.01],
[0.39,0.30,0.25,0.21,0.17,0.13,0.09,0.03,0.53,0.50],
[0.18,0.08,0.02,0.53,0.52,0.50,0.48,0.45,0.42,0.38]
]

line_names = [r"$\omega=1$", r"$\omega=3$", r"$\omega=5$", r"$\omega=7$", r"$\omega=10$"]
x_label = 'Starting Point of the FLS'
y_label = '% Improvement in Time by FRT'
# draw_line_chart_percentage(y_lists, x, line_names, x_label, y_label, save_name='../results/improve_time.png')

save_name = '../results/improve_time.png'
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)
for i, data in enumerate(zip(y_lists, line_names, colors, markers)):
    y, line_name, color, marker = data
    ax.plot(x, y, marker=marker, linestyle='-', color=color, label=line_name)
    if i == 0:
        ax.text((x[1] + x[1])/2, y[1] + (max(max((y_lists)))-min(min(y_lists)))/10, line_name, fontsize=28, ha='center',
                va='center', color=color, font='Times New Roman')
    elif i == 1:
        ax.text((x[1] + x[1])/2, y[1] + (max(max((y_lists)))-min(min(y_lists)))/20, line_name, fontsize=28, ha='center',
                va='center', color=color, font='Times New Roman')

    elif i == 2:
        ax.text((x[1] + x[1]) / 2, y[1] + (max(max((y_lists))) - min(min(y_lists))) / 15, line_name, fontsize=28,
                ha='center', va='center', color=color, font='Times New Roman')
    elif i == 3:
        ax.text((x[1] + x[1]) / 2, y[1] + (max(max((y_lists))) - min(min(y_lists))) / 15, line_name, fontsize=28,
                ha='center', va='center', color=color, font='Times New Roman')
    else:
        ax.text((x[1] + x[1])/2 + 0.1, y[1] + (max(max((y_lists)))-min(min(y_lists)))/12, line_name, fontsize=28, ha='center', va='center', color=color, font='Times New Roman')


# Add title and labels
ax.set_xlabel(x_label, fontsize=30, font='Times New Roman')
title_font = {'fontname': 'Times New Roman', 'fontsize': 30}
# ax.set_title(y_label, fontdict=title_font, loc='left', pad=0)
ax.text(0.2, 1.05, y_label, fontdict=title_font)

# ax.set_ylabel(y_label, fontsize=16, font='Times New Roman')

def to_percent(y, position):
    # Convert y values to percentage
    s = f"{100 * y:.0f}"
    return s + '%'

# Set the y-axis formatter to percentage
ax.yaxis.set_major_formatter(FuncFormatter(to_percent))

ax.spines[['right', 'top']].set_visible(False)

for label in (ax.get_xticklabels() + ax.get_yticklabels()):
    label.set_fontname('Times New Roman')
    label.set_fontsize(28)

ax.set_ylim(0, 1)
# Show the plot
ax.grid(True)
plt.tight_layout()

if save_name is not None:
    fig.savefig(save_name, dpi=300)
fig.show()



# Improvement of dist
x = [i for i in range(1, 11)]
y_lists = [
    [0.28, 0.55, 0.53, 0.43, 0.33, 0.25, 0.17, 0.11, 0.06, 0.02],
    [0.13, 0.21, 0.26, 0.25, 0.21, 0.17, 0.12, 0.07, 0.03, 0.01],
    [0.07, 0.09, 0.11, 0.11, 0.10, 0.07, 0.05, 0.03, 0.01, 0.00],
    [0.04, 0.03, 0.03, 0.03, 0.02, 0.02, 0.01, 0.00, 0.00, 0.01],
    [0.01, 0.00, 0.00, 0.00, 0.00, 0.01, 0.02, 0.03, 0.05, 0.07],
]

line_names = [r"$\omega=1$", r"$\omega=3$", r"$\omega=5$", r"$\omega=7$", r"$\omega=10$"]
x_label = 'Starting Point of the FLS'
y_label = '% Improvement in Distance by SD'
# draw_line_chart_percentage(y_lists, x, line_names, x_label, y_label, save_name='../results/improve_dist.png')

save_name='../results/improve_dist.png'

fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)
for i, data in enumerate(zip(y_lists, line_names, colors, markers)):
    y, line_name, color, marker = data
    ax.plot(x, y, marker=marker, linestyle='-', color=color, label=line_name)
    if i == 4:
        ax.text((x[9] + x[8])/2 + 0.2, y[9] + (max(max((y_lists)))-min(min(y_lists)))/15, line_name, fontsize=28, ha='center',
                va='center', color=color, font='Times New Roman')
    elif i == 0:
        ax.text((x[1] + x[1])/2, y[1] + (max(max((y_lists)))-min(min(y_lists)))/15, line_name, fontsize=28, ha='center',
                va='center', color=color, font='Times New Roman')
    elif i == 1:
        ax.text((x[1] + x[1])/2, y[1] + (max(max((y_lists)))-min(min(y_lists)))/8, line_name, fontsize=28, ha='center',
                va='center', color=color, font='Times New Roman')
    elif i == 3:
        ax.text((x[1] + x[1]) / 2, y[1] + (max(max((y_lists))) - min(min(y_lists))) / 25, line_name, fontsize=28,
                ha='center', va='center', color=color, font='Times New Roman')
    else:
        ax.text((x[1] + x[1])/2, y[1] + (max(max((y_lists)))-min(min(y_lists)))/15, line_name, fontsize=28, ha='center', va='center', color=color, font='Times New Roman')


# Add title and labels
ax.set_xlabel(x_label, fontsize=30, font='Times New Roman')
title_font = {'fontname': 'Times New Roman', 'fontsize': 30}
# ax.set_title(y_label, fontdict=title_font, loc='left', pad=0)
ax.text(0.2, 1.05, y_label, fontdict=title_font)

# ax.set_ylabel(y_label, fontsize=16, font='Times New Roman')

def to_percent(y, position):
    # Convert y values to percentage
    s = f"{100 * y:.0f}"
    return s + '%'

# Set the y-axis formatter to percentage
ax.yaxis.set_major_formatter(FuncFormatter(to_percent))

ax.spines[['right', 'top']].set_visible(False)

for label in (ax.get_xticklabels() + ax.get_yticklabels()):
    label.set_fontname('Times New Roman')
    label.set_fontsize(28)

ax.set_ylim(0, 1)
# Show the plot
ax.grid(True)
plt.tight_layout()

if save_name is not None:
    fig.savefig(save_name, dpi=300)
fig.show()
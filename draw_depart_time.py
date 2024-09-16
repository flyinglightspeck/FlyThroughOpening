from collections import Counter

from util import *

files = ["/Users/shuqinzhu/Desktop/collisions_time/kangaroo_972_Q10_S1.5_N_25_P1_time.csv",
         "/Users/shuqinzhu/Desktop/collisions_time/kangaroo_972_Q10_S1.5_N_25_P0_time.csv",
         "/Users/shuqinzhu/Desktop/collisions_time/kangaroo_972_Q10_S1.5_N_25_OPT_time.csv",
         ]

datas = []
x_lists = []
y_lists = []


makespans = [int(ceil(30175/30)), int(ceil(24391/30)), int(ceil(10565/30))]

for file_path in files:
    df = pd.read_csv(file_path)
    data = df.iloc[:, 2]
    datas.append(list(data))


for data in datas:
    value_counts = Counter(data)

    # Sort the data by value (X-axis should be the value range)
    x_values = sorted(value_counts.keys())
    y_values = [value_counts[value] for value in x_values]

    x_lists.append(x_values)
    y_lists.append(y_values)

save_name = './results/departure_makespan_cmp.png'

x_label = "Time (Second)"
y_label = "Number of FLSs Depart at the Time"

markers = ['o', 'o', 'o', 'D', 'h', 'v']
colors = ["#0072BD", "#7E2F8E", "#8B6C5C", "#edb120", "#000000", "#4DBEEE"]

line_names = ["SD", "FRT", "OPT"]

fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111)


text_positions = [[900, 4], [600, 4], [200, 4]]
makespan_text_positions = [[910, 15], [712, 15], [255, 15]]
linewidth_list = [5, 4, 3]
for i, value in enumerate(zip(x_lists, y_lists, line_names, colors, markers)):
    # if i == 0:
    #     continue
    x, y, line_name, color, marker = value
    ax.plot(x, y, marker=marker, linestyle='-', color=color, label=line_name, linewidth=linewidth_list[i], markerfacecolor='white', markersize=8, markeredgewidth=3)

    ax.text(text_positions[i][0], text_positions[i][1], line_name, fontsize=28,
                ha='center', va='center', color=color, font='Times New Roman')

    ax.axvline(x=makespans[i], color=color, linestyle='--', linewidth=3)
    ax.text(makespan_text_positions[i][0], makespan_text_positions[i][1], f"Makespan\n of {line_name}", fontsize=26,
            ha='center', va='center', color=color, font='Times New Roman')

# Add title and labels
ax.set_xlabel(x_label, fontsize=30, font='Times New Roman')
title_font = {'fontname': 'Times New Roman', 'fontsize': 30}
ax.set_title(y_label, fontdict=title_font, loc='left', pad=0)
# ax.set_ylabel(y_label, fontsize=16, font='Times New Roman')

# Add a legend
# plt.legend()

# ax.set_ylim(0, 16)
ax.spines[['right', 'top']].set_visible(False)
# Show the plot
ax.grid(True)
for label in (ax.get_xticklabels() + ax.get_yticklabels()):
    label.set_fontname('Times New Roman')
    label.set_fontsize(28)

plt.tight_layout()
if save_name is not None:
    fig.savefig(save_name, dpi=300)
fig.show()



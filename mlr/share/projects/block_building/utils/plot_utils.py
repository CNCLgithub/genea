import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np

from scipy import stats


COLOR_LIST = ["#069285", "#701b1b", "#06454e", "#b15c47", "#79b5e0", "#6ed5c0"]


class PlotUtils:
    @staticmethod
    def draw_scatter_plot(x_list, y_list, title, x_label, y_label, annot=None, save_path=None):
        x_list = np.array(x_list)
        y_list = np.array(y_list)

        if len(x_list) == 0 or len(y_list) == 0:
            return

        sns.set_theme(style='whitegrid')
        sns.set_palette("pastel")
        sns.color_palette("RdPu", 20)

        x_min = min(x_list) - 0.1
        x_max = max(x_list) + 0.1
        y_min = min(y_list) - 2
        y_max = max(y_list) + 2

        # x_min = -0.02
        # x_max = 1.02
        # y_min = -2
        # y_max = 102

        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)
        # plt.grid(b=None)

        slope, intercept, r, p, _ = stats.linregress(x=x_list, y=y_list)

        scatter_kws = {"color": "#12314e", "linewidth": 0, "s": 200, "zorder": 10, 'clip_on': False}

        sns.regplot(x=x_list, y=y_list, line_kws={"color": "#a8a4a4"}, scatter_kws=scatter_kws, fit_reg=True)
        sns.despine()
        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)

        for i in range(len(x_list)):
            if annot:
                plt.annotate(annot[i], (x_list[i], y_list[i]))

        # plt.text(min(x_list), np.ceil(max(y_list)), 'r=' + str(np.mean(r)) + "; p=" + str(p), ha='left', va='top')

        if save_path:
            plt.savefig(save_path)
            plt.close()
        else:
            plt.show()
            plt.close()

    @staticmethod
    def draw_bar_plot(x_labels_list, y_values_list):
        sns.set_theme(style="whitegrid")
        sns.barplot(x=x_labels_list, y=y_values_list, capsize=.1, ci="sd")
        plt.show()
        plt.close()

    @staticmethod
    def draw_multiple_bar_plots(x_labels_list, y_values_list_dict, behavior_values_list=None, save_path=None):
        # color_list = ["#12314e", "#505052", "#666668", "#929498", "#ACACAE"]
        color_list = ["#12314e", "#505052", "#12314e", "#505052", "#12314e", "#505052", "#12314e", "#505052"]

        plt.rcParams['legend.handlelength'] = 1
        plt.rcParams['legend.handleheight'] = 1.125

        sns.set_theme(style="whitegrid")

        split_half_lower, split_half_upper = behavior_values_list

        means_list = [np.mean(value) for key, value in y_values_list_dict.items()]
        lower_list = [np.mean(value) - np.percentile(value, 2.5) for key, value in y_values_list_dict.items()]
        upper_list = [np.percentile(value, 97.5) - np.mean(value) for key, value in y_values_list_dict.items()]
        y_err_list = [lower_list, upper_list]

        ax = sns.barplot(x=np.arange(len(means_list)), y=means_list,
                         yerr=y_err_list, palette=color_list, label=x_labels_list)

        ax.axhline(y=split_half_lower, color="#929498", linestyle="--", linewidth=.5)
        ax.axhline(y=split_half_upper, color="#929498", linestyle="--", linewidth=.5)

        ax.yaxis.grid(False)
        ax.xaxis.grid(False)
        ax.set_axisbelow(True)
        ax.set_xticklabels(x_labels_list)
        plt.xticks(rotation=90)
        ax.tick_params(labelsize=5)
        plt.ylabel("Correlation", fontsize=20)
        plt.tight_layout()
        # plt.legend(fontsize=20, loc='upper center', bbox_to_anchor=(0.5, 1.05), fancybox=True, shadow=True, ncol=5)

        if save_path:
            plt.savefig(save_path)
            plt.close()
        else:
            plt.show()
            plt.close()

    @staticmethod
    def draw_strip_plot(x_list, y_list, x_model_list, y_model_list, save_path=None):
        sns.set_theme(style='whitegrid')
        sns.set_palette("pastel")
        sns.color_palette("RdPu", 20)

        box_color_props = {
            'boxprops': {'facecolor': "#ffffff", 'edgecolor': "#bebebe"},
            'medianprops': {'color': "#bebebe"},
            'whiskerprops': {'color': "#bebebe"},
            'capprops': {'color': "#bebebe"}
        }

        # sns.stripplot(x=x_list, y=y_list, orient="v", zorder=0, color="#bebebe", jitter=0.2, size=10)
        sns.boxplot(x=x_list, y=y_list, orient="v", width=0.5, zorder=0, color="#bebebe", **box_color_props)
        sns.pointplot(x=x_list, y=y_list, join=False, color="#79b5e0", ci=95, zorder=1, label="human", sizes=[30])

        sns.pointplot(x=x_model_list, y=y_model_list, join=False, color="#12314e", sizes=[30])
        plt.ylim(-10.0, 110.0)
        plt.yticks(range(-10, 120, 10), range(-10, 120, 10))
        plt.grid(False)
        plt.xticks(size=10, rotation=90)
        plt.tick_params(axis='y', which='both', left=True, right=False)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path)
            plt.close()
        else:
            plt.show()
            plt.close()

    @staticmethod
    def action_goal_custom_plot(x_labels_list, means_list, y_err_list, colors_list, save_path=None):
        plt.rcParams['legend.handlelength'] = 1
        plt.rcParams['legend.handleheight'] = 1.125

        sns.set_theme(style="whitegrid")

        ax = sns.barplot(x=x_labels_list, y=means_list, yerr=y_err_list, palette=colors_list)

        ax.yaxis.grid(False)
        ax.xaxis.grid(False)
        ax.set_axisbelow(True)
        ax.tick_params(labelsize=10, rotation=90)
        plt.ylabel("mean score", fontsize=10)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path)
            plt.close()
        else:
            plt.show()
            plt.close()

    @staticmethod
    def draw_blocks_table_plot(block_vertices_list, colors_list, names_list, is_diff=False):
        plt.figure(dpi=1000)

        for idx, (vertices, color, b_name) in enumerate(zip(block_vertices_list, colors_list, names_list)):
            vertices_closed = np.vstack([vertices, vertices[0]])
            if is_diff:
                plt.plot(-vertices_closed[:, 0], vertices_closed[:, 1], '-', linewidth=0.1, color=color)
                centroid_x = -np.mean(vertices_closed[:, 0])
                centroid_y = np.mean(vertices_closed[:, 1])
            else:
                plt.plot(vertices_closed[:, 0], vertices_closed[:, 1], '-', linewidth=0.1, color=color)
                centroid_x = np.mean(vertices_closed[:, 0])
                centroid_y = np.mean(vertices_closed[:, 1])

            plt.text(centroid_x, centroid_y, b_name, fontsize=8, color='black', ha='center', va='center')

        plt.tight_layout()
        plt.tick_params(width=0.1, length=0.1, which='both')
        plt.xticks(np.round(np.arange(-.4, .45, 0.05), 2), np.round(np.arange(-.4, .45, 0.05), 2), size=1)
        plt.yticks(np.round(np.arange(-.1, .35, 0.05), 2), np.round(np.arange(-.1, .35, 0.05), 2), size=1)
        if is_diff:
            plt.xticks(np.round(np.arange(-.8, .85, 0.05), 2), np.round(np.arange(-.8, .85, 0.05), 2), size=1)
            plt.yticks(np.round(np.arange(-.4, .45, 0.05), 2), np.round(np.arange(-.4, .45, 0.05), 2), size=1)
        plt.show()

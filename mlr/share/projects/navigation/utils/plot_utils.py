import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

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
        plt.grid(True)

        plt.xlim(min(x_list), max(x_list))
        plt.ylim(min(y_list), max(y_list))

        slope, intercept, r, p, _ = stats.linregress(x=x_list, y=y_list)

        scatter_kws = {"color": "#12314e", "linewidth": 0, "s": 200, "zorder": 10}

        sns.regplot(x=x_list, y=y_list, line_kws={"color": "#a8a4a4"}, scatter_kws=scatter_kws, fit_reg=True)
        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)

        for i in range(len(x_list)):
            if annot:
                plt.annotate(annot[i], (x_list[i], y_list[i]))

        if save_path:
            plt.savefig(save_path)
            plt.close()
        else:
            plt.show()
            plt.close()

    @staticmethod
    def draw_histogram_plot(samples_list,  bins, title, x_label, y_label):
        plt.figure()
        plt.hist(samples_list, bins=bins, density=True)
        plt.title(title)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.show()

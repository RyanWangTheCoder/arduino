import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from scipy import signal, datasets
from scipy.signal import butter, lfilter, freqz
from datetime import datetime

class EMGDataHelper:
    def __init__(self, data_file):
        self.dataFile = data_file
        self.data = pd.read_csv(self.dataFile, sep=' --> ', header=None, names=["time", "value"], engine='python')
        print(self.data)
        for index, row in self.data.iterrows():
            t = self.data.loc[index, 'time']
            self.data.iloc[index, 0] = datetime.strptime(t, '%H:%M:%S.%f').time()
            self.data.iloc[index, 1] = float(self.data.loc[index, 'value'])
        print(self.data)
        self.x = np.linspace(1, len(self.data), num=len(self.data))
        self.dataValue = self.data['value']
# reading the database
    def doInterp(self):
        spl = CubicSpline(self.x, self.dataValue)

        fig, ax = plt.subplots(4, 1)
        plt.grid()

        # Scatter plot with day against tip
        #ax[0].plot(self.x, spl(self.x))
        ax[0].plot(self.x, self.dataValue, 'o', label='data')
        plt.title("EMG Plot")
        ax[1].plot(self.x, spl(self.x, nu=1), '--', label='1st derivative')
        ax[2].plot(self.x, spl(self.x, nu=2), '--', label='2nd derivative')
        ax[3].plot(self.x, spl(self.x, nu=3), '--', label='3rd derivative')
        # Adding Title to the Plot
        ax[0].title.set_text('EMG Plot')
        ax[1].title.set_text('Second Plot')
        ax[2].title.set_text('Third Plot')
        ax[3].title.set_text('Fourth Plot')
        for ax_idx in ax.flat:
            ax_idx.grid(True)
        # Setting the X and Y labels
        #plt.xlabel('Day')
        #plt.ylabel('Tip')
        plt.legend()

        plt.subplots_adjust(hspace=0.35)
        plt.show()

        return

    def butter_lowpass(self, cutoff, fs, order=5):
        return butter(order, cutoff, fs=fs, btype='low', analog=False)

    def butter_lowpass_filter(self, data, cutoff, fs, order=5):
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y
    def doFilter(self):
        # Filter requirements.
        order = 6
        fs = 30.0  # sample rate, Hz
        cutoff = 3.667  # desired cutoff frequency of the filter, Hz

        # Get the filter coefficients so we can check its frequency response.
        b, a = self.butter_lowpass(cutoff, fs, order)

        # Plot the frequency response.
        w, h = freqz(b, a, fs=fs, worN=8000)
        plt.subplot(2, 1, 1)
        plt.plot(w, np.abs(h), 'b')
        plt.plot(cutoff, 0.5 * np.sqrt(2), 'ko')
        plt.axvline(cutoff, color='k')
        plt.xlim(0, 0.5 * fs)
        plt.title("Lowpass Filter Frequency Response")
        plt.xlabel('Frequency [Hz]')
        plt.grid()

        # Demonstrate the use of the filter.
        # First make some data to be filtered.
        # Filter the data, and plot both the original and filtered signals.
        y = self.butter_lowpass_filter(self.dataValue, cutoff, fs, order)

        plt.subplot(2, 1, 2)
        plt.plot(self.x, self.dataValue, 'b-', label='data')
        plt.plot(self.x, y, 'g-', linewidth=2, label='filtered data')
        plt.xlabel('Time [sec]')
        plt.grid()
        plt.legend()

        plt.subplots_adjust(hspace=0.35)
        plt.show()
class GSRDataHelper:
    def __init__(self, data_file):
        self.dataFile = data_file
        self.data = pd.read_csv(self.dataFile, sep=' --> ', header=None, names=["time", "value"], engine='python')
        print(self.data)
        for index, row in self.data.iterrows():
            t = self.data.loc[index, 'time']
            self.data.iloc[index, 0] = datetime.strptime(t, '%H:%M:%S.%f').time()
            self.data.iloc[index, 1] = float(self.data.loc[index, 'value'])
        print(self.data)


if __name__ == '__main__':
    emgData = EMGDataHelper("C:\\Users\\ryanw\\OneDrive\\Documents\\Arduino\\new.txt")
    emgData.doInterp()
    emgData.doFilter()
    #gsrData = GSRDataHelper("C:\\Users\\ryanw\\OneDrive\\Documents\\Arduino\\new.txt")

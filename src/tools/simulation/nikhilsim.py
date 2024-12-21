import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox
from stl import mesh
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Rocket3DApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("3D Rocket Visualization")
        self.geometry("800x600")

        self.load_button = tk.Button(self, text="Load STL Rocket Model", command=self.load_stl)
        self.load_button.pack(pady=10)

        self.figure = plt.figure(figsize=(8, 6))
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.figure, master=self)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def load_stl(self):
        file_path = filedialog.askopenfilename(filetypes=[("STL Files", "*.stl")])
        if not file_path:
            return 

        try:
            rocket_mesh = mesh.Mesh.from_file(file_path)

            self.ax.clear()

            self.plot_stl(rocket_mesh)

            self.canvas.draw()
        except Exception as e:
            messagebox.showerror("Error", f"Could not load STL file:\n{e}")

    def plot_stl(self, rocket_mesh):
        min_bound = np.min(rocket_mesh.vectors, axis=(0, 1))
        max_bound = np.max(rocket_mesh.vectors, axis=(0, 1))
        max_range = np.max(max_bound - min_bound) / 2.0

        mid_x = (max_bound[0] + min_bound[0]) / 2.0
        mid_y = (max_bound[1] + min_bound[1]) / 2.0
        mid_z = (max_bound[2] + min_bound[2]) / 2.0

        self.ax.set_xlim(mid_x - max_range, mid_x + max_range)
        self.ax.set_ylim(mid_y - max_range, mid_y + max_range)
        self.ax.set_zlim(mid_z - max_range, mid_z + max_range)

        faces = []
        for face in rocket_mesh.vectors:
            v0, v1, v2 = face
            if len({tuple(v0), tuple(v1), tuple(v2)}) < 3:
                continue
            faces.append([v0, v1, v2])

        poly3d_collection = Poly3DCollection(faces, color="cyan", edgecolor="k", linewidths=0.2, alpha=0.8)
        self.ax.add_collection3d(poly3d_collection)

        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title("Rocket Model")

if __name__ == "__main__":
    app = Rocket3DApp()
    app.mainloop()

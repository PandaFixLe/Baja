# scripts/plot_traj.py
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('build/trajectory.csv')
ref = df[df['type']=='ref']
car = df[df['type']=='car']

plt.figure(figsize=(10,6))
plt.plot(ref['x'], ref['y'], 'b--', label='Reference Path', linewidth=2)
plt.plot(car['x'], car['y'], 'r-', label='Car Trajectory', linewidth=1)
plt.axis('equal')
plt.grid(True)
plt.legend()
plt.title('Baja Autonomous Tracking Performance')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.savefig('traj_plot.png', dpi=300)
plt.show()
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def read_and_plot_velocities(csv_file):
    try:
        print(f"Đang đọc file: {csv_file}")
        df = pd.read_csv(csv_file)
        print(f"Các cột trong file CSV: {df.columns.tolist()}")
        time_col = None 
        velocity_col = None  
        omega_col = None
        
        for col in df.columns:
            if col.lower() == 'time':
                time_col = col
            elif col.lower() == 'velocity':
                velocity_col = col
            elif col.lower() == 'omega':
                omega_col = col
        print(f"Cột thời gian: {time_col}")
        print(f"Cột vận tốc tuyến tính: {velocity_col}")
        print(f"Cột vận tốc góc: {omega_col}")
        if not all([time_col, velocity_col, omega_col]):
            print("Lỗi: Không tìm thấy tất cả các cột cần thiết trong file CSV.")
            if len(df.columns) >= 3:
                print("Sử dụng cột theo vị trí: cột 1 = thời gian, cột 2 = vận tốc tuyến tính, cột 3 = vận tốc góc")
                time_col = df.columns[0]
                velocity_col = df.columns[1]
                omega_col = df.columns[2]
            else:
                raise ValueError("File CSV không có đủ 3 cột dữ liệu")
        plt.figure(figsize=(12, 10))

        plt.subplot(2, 1, 1)
        plt.plot(df[time_col], df[velocity_col], 'b-', linewidth=2)
        plt.title('Vận tốc tuyến tính theo thời gian', fontsize=14)
        plt.xlabel('Thời gian', fontsize=12)
        plt.ylabel('Vận tốc tuyến tính', fontsize=12)
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        plt.plot(df[time_col], df[omega_col], 'r-', linewidth=2)
        plt.title('Vận tốc góc theo thời gian', fontsize=14)
        plt.xlabel('Thời gian', fontsize=12)
        plt.ylabel('Vận tốc góc', fontsize=12)
        plt.grid(True)
        plt.tight_layout()
        output_file = 'velocity_graphs_from_csv.png'
        plt.savefig(output_file)
        print(f"Đồ thị đã được lưu vào file: {output_file}")
        plt.show()
        
    except Exception as e:
        print(f"Đã xảy ra lỗi: {e}")

if __name__ == "__main__":
    csv_path = "velocity_omega_profile.csv"
    
    if not csv_path:
        csv_path = "velocity_data.csv"
    if os.path.exists(csv_path):
        read_and_plot_velocities(csv_path)
    else:
        print(f"Lỗi: File '{csv_path}' không tồn tại.")
        print("\nTạo file CSV mẫu 'velocity_data.csv' để kiểm tra...")
        time = np.arange(0, 10, 0.01)
        velocity = 2 * np.sin(time) + 0.5 * time
        omega = np.cos(time) * 0.5
        sample_df = pd.DataFrame({
            'Time': time,
            'Velocity': velocity,
            'Omega': omega
        })
        sample_df.to_csv('velocity_data.csv', index=False)
        print("Đã tạo file CSV mẫu 'velocity_data.csv'")
        read_and_plot_velocities('velocity_data.csv')
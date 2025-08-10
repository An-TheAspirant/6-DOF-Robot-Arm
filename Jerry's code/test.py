import numpy as np

class trans_mat_update():
    def __init__(self, number_of_joints, DPL):
        self.consecutive_joint_matrix = []
        self.mult_joint_matrix = []
        self.DH_par_list = DPL
        self.n_joints = number_of_joints

    def DH_par_input(self):
        for i in range(self.n_joints):
            indi_matrix = list(map(int, input(f"MATRIX NUMBER {i+1}: ").split()))
            self.DH_par_list.append(indi_matrix)

    def DH_par_update(self, new_DH_list):
        self.DH_par_list = new_DH_list

    def generate_matrices(self):
        if self.n_joints == 0:
            print("No joints created")
            return
        if len(self.DH_par_list) == 0:
            print("DH_par list emptied")
            return

        self.consecutive_joint_matrix = []
        self.mult_joint_matrix = []
        for i in range(self.n_joints):
            r = self.DH_par_list[i][0] #delta z/x_latest
            alpha = np.radians(self.DH_par_list[i][1]) #angle between z
            d = self.DH_par_list[i][2] #delta x/z_latest
            omega = np.radians(self.DH_par_list[i][3]) #angle between x
            omega = np.where(abs(omega) < 0.001, 0 ,omega)
            homo_mat = [[np.cos(omega), -np.sin(omega)*np.cos(alpha),  np.sin(omega)*np.sin(alpha),  r*np.cos(omega)],
                           [np.sin(omega),  np.cos(omega)*np.cos(alpha), -np.cos(omega)*np.sin(alpha),  r*np.sin(omega)],
                           [0,              np.sin(alpha),                np.cos(alpha),                d],
                           [0,              0,                            0,                            1]]
            homo_mat = np.array(homo_mat)
            homo_mat = np.where(abs(homo_mat) < 0.0001, 0, homo_mat)
            self.consecutive_joint_matrix.append(homo_mat)
            if i == 0:
                self.mult_joint_matrix.append(homo_mat)
            else: self.mult_joint_matrix.append(np.dot(self.mult_joint_matrix[i-1], homo_mat))

    def update_pos_txt(self):
        f = open("Pos.txt", "w")
        f.truncate(0)
        for i in range(len(self.mult_joint_matrix)):
            for j in range(0, 3):
                f.write(str(self.mult_joint_matrix[i][j][-1]))
                if j != 2:
                    f.write(" ")
                else:
                    f.write("\n")
        f.close()

    def GET_cons_joint_matrix(self):
        return np.array(self.consecutive_joint_matrix)
    def GET_multiplicated_joint_matrix(self):
        return np.array(self.mult_joint_matrix)

DH_par_lists = [[0, 90, 10, 0],
                [10, 0, 0, 0],
                [10, 0, 0, 0],
                [5, 0, 0, 0]]

model = trans_mat_update(4, DH_par_lists)
model.generate_matrices()
print(model.consecutive_joint_matrix)
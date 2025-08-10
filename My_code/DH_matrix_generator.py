import numpy as np

class TransMatUpdate:
    def __init__(self, number_of_joints, DPL=None):
        """
        Initializes the class with the number of joints and DH parameter list.
        """
        self.consecutive_joint_matrix = []
        self.mult_joint_matrix = []
        self.DH_par_list = DPL if DPL else []
        self.n_joints = number_of_joints

    def dh_par_input(self):
        """
        Prompts the user to input DH parameters for each joint.
        """
        for i in range(self.n_joints):
            indi_matrix = list(map(int, input(f"MATRIX NUMBER {i+1}: ").split()))
            self.DH_par_list.append(indi_matrix)

    def dh_par_update(self, new_DH_list):
        """
        Updates the DH parameter list.
        """
        self.DH_par_list = new_DH_list

    def generate_matrices(self, precision_threshold=0.0001):
        """
        Generates the consecutive and cumulative transformation matrices based on DH parameters.
        """
        if self.n_joints <= 0:
            print("Error: Number of joints must be greater than zero.")
            return
        if not self.DH_par_list or len(self.DH_par_list) != self.n_joints:
            print("Error: DH parameter list is empty or does not match the number of joints.")
            return

        self.consecutive_joint_matrix = []
        self.mult_joint_matrix = []
        try:
            for i in range(self.n_joints):
                r = self.DH_par_list[i][0]
                alpha = np.radians(self.DH_par_list[i][1])
                d = self.DH_par_list[i][2]
                omega = np.radians(self.DH_par_list[i][3])
                omega = np.where(abs(omega) < 0.001, 0, omega)

                homo_mat = [[np.cos(omega), -np.sin(omega)*np.cos(alpha),  np.sin(omega)*np.sin(alpha),  r*np.cos(omega)],
                            [np.sin(omega),  np.cos(omega)*np.cos(alpha), -np.cos(omega)*np.sin(alpha),  r*np.sin(omega)],
                            [0,              np.sin(alpha),                np.cos(alpha),                d],
                            [0,              0,                            0,                            1]]
                homo_mat = np.array(homo_mat)
                homo_mat = np.where(abs(homo_mat) < precision_threshold, 0, homo_mat)

                self.consecutive_joint_matrix.append(homo_mat)
                if i == 0:
                    self.mult_joint_matrix.append(homo_mat)
                else:
                    self.mult_joint_matrix.append(np.dot(self.mult_joint_matrix[i-1], homo_mat))
        except Exception as e:
            print(f"Error generating matrices: {e}")

    def update_pos_txt(self, file_name="Pos.txt"):
        """
        Writes the end-effector positions to a text file.
        """
        try:
            with open(file_name, "w") as f:
                for i in range(len(self.mult_joint_matrix)):
                    for j in range(0, 3):
                        f.write(str(self.mult_joint_matrix[i][j][-1]))
                        if j != 2:
                            f.write(" ")
                        else:
                            f.write("\n")
        except IOError as e:
            print(f"Error writing to file {file_name}: {e}")

    def get_consecutive_joint_matrix(self):
        """
        Returns the consecutive joint matrices as a NumPy array.
        """
        return np.array(self.consecutive_joint_matrix)

    def get_multiplicated_joint_matrix(self):
        """
        Returns the cumulative joint matrices as a NumPy array.
        """
        return np.array(self.mult_joint_matrix)


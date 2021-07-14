

class Env:
    def __init__(self):
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    '''
    ####concave
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [45,22,5,1],
            [45, 22, 3, 4],
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
        ]

        return obs_cir
        '''




    '''
    ####narrow
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [20, 1, 7, 13.7],
            [20, 17, 7, 13],
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [40,16,5]
        ]

        return obs_cir
        '''








    ####first one
    

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2],
            
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3],
        ]

        return obs_cir









    '''
    ###empty
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [

        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
        ]

        return obs_cir
        '''










    '''
    ###easy
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [5, 5, 8, 3],
            [13, 20, 10, 5],
            [39, 21, 6, 4]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [35, 11, 3.5],
        ]

        return obs_cir
        '''








    '''
    ###normal
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [4.5, 18, 7, 7],
            [13, 12, 8, 2],
            [17, 22, 8, 3],
            [28, 7, 2, 12],
            [33, 12, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [6, 7, 3],
            [45.5, 19, 2],
            [16, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

        return obs_cir
        '''









    ''''
    ###dense
    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [4.5, 18, 7, 7],
            [11, 7, 2, 3],
            [6.5, 11.5, 2, 4],
            [13, 12, 8, 2],
            [17, 22, 8, 3],
            [28, 7, 4, 12],
            [12.5, 15.5, 2.5, 7],
            [20, 5, 6, 5],
            [40, 25, 4, 3],
            [42.5, 3, 4, 5],
            [33, 12, 10, 4],
            [41, 21.5, 3.5, 2],
            [22, 1, 8, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [6, 7, 3],
            [24, 12.5, 2],
            [14, 27, 2],
            [45.5, 19, 2],
            [16, 5, 2],
            [23, 18, 3],
            [46, 12, 2],
            [29.3, 26.5, 2],
            [37, 7, 3],
            [37, 21, 3]
        ]

        return obs_cir
        '''



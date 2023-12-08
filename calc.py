import math

# Входные данные
m = 132  # массы кабеля в кг/км;
g = 9.8  # ускорения свободного падения в м/с^2;
rho_l = 0.0009  # объемная масса гололеда (обычно 0,9·10-3), кг/см3;
k_i = 1  # Коэффициенты учитывающий изменение толщины стенки гололеда по высоте над поверхностью земли;
k_d = 1  # коэффициенты учитывающий изменение толщины стенки гололеда в зависимости от диаметра провода;
k_l = 1.05  # коэффициент, учитывающий влияние длины пролета на ветровую нагрузку;
k_w = 1  # коэффициент, учитывающий изменение ветрового давления по высоте в зависимости от типа местности;
C = 15  # толщина стенки гололеда, мм;
d = 12.8  # диаметр кабеля, мм;
alpha_w = 0.71  # коэффициент, учитывающий неравномерность ветрового давления по пролету ВЛ;
C_x = 1.2  # коэффициент лобового сопротивления;
d_kab = 12.8  # внешний диаметр кабеля, мм;
w_0 = 500  # нормативное ветровое давление, Па;
L = 130  # расстояние между опорами, м;
h = 0  # перепад высот между точками подвеса кабеля, м.
T = 10  # температура кабеля в условиях эксплуатации;
T_sr = 10  # средняя температура эксплуатации.
TkLR = 24.6  # температурным коэффициентом линейного расширения кабеля
E_kab = 5.22

S = 1.2


# 1.2. Сечение кабеля
def calculate_h(d_kab):
    return (math.pi * d_kab ** 2) / 4


s_kab = calculate_h(d_kab)


# 2.1. Вес кабеля.
# w_{каб}=\frac{m*g}{1000}
def calculate_w_kab(m, g):
    return m * g / 1000


w_kab = calculate_w_kab(m, g)
w = w_kab


# 2.2. Растягивающая нагрузка, действующая на кабель.
# H=\frac{W*L^{2}}{8*S}
def calculate_h(w, L, S):
    return (w * L ** 2) / (8 * S)


H = calculate_h(w, L, S)

H_start = H


# 2.3. Перепад высот между опорами.

# L_{1} = L - \frac{2*h*H_{нач}}{W*L}
def calculate_L1(L, h, H_start, w):
    return L - (2 * h * H_start) / (w * L)


L1 = calculate_L1(L, h, H_start, w)


# L_{2} = L + \frac{2*h*H_{нач}}{W*L}
def calculate_L2(L, h, H_start, w):
    return L + (2 * h * H_start) / (w * L)


L2 = calculate_L2(L, h, H_start, w)


# S_{1} = \frac{W-W^{2}_{1}}{8*H}
def calculate_S1(w, L1, H):
    return (w * L1 ** 2) / (8 * H)


S1 = calculate_S1(w, L1, H)


# S_{2} = \frac{W*L^{2}_{2}}{8*H}
def calculate_S2(w, L2, H):
    return (w * L2 ** 2) / (8 * H)


S2 = calculate_S2(w, L2, H)


# 2.4. Длина подвешенного кабеля.


# L_{каб}=Д+\frac{4}{3}*(\frac{S^{2}_{1}}{L_{1}}+\frac{S^{2}_{2}}{L_{2}})

def calculate_L_kab(L, S1, L1, S2, L2):
    return L + (4 / 3) * ((S1 ** 2 / L1) + (S2 ** 2 / L2))


L_kab = calculate_L_kab(L, S1, L1, S2, L2)


# 2.5. Длина кабеля в ненагруженном состоянии:
# L_{н0}=\frac{L_{каб}}{1+(\frac{H}{E_{каб}*S_{каб}})}

def calculate_l_no(L_kab, H, E_kab, s_kab):
    return L_kab / (1 + (H / (E_kab * s_kab)))


L_no = calculate_l_no(L_kab, H, E_kab, s_kab)


# 2.6. Длина кабеля в ненагруженном состоянии с учетом температуры:
# L_{нк}=L_{но}*\left[ 1+TKLR*(T-T_{ср}) \right]
def calculate_l_nk(L_no, TkLR, T, T_sr):
    return L_no * (1 + TkLR * (T - T_sr))


L_nk = calculate_l_nk(L_no, TkLR, T, T_sr)


# 2.7. Вес кабеля при воздействии максимального гололеда.
# w_{г}=w+\rho_{л}*g*\pi*k_{i}*k_{d}*C*(d+C)
def calculate_w_g(w, rho_l, g, k_i, k_d, C, d):
    return w + rho_l * g * math.pi * k_i * k_d * C * (d + C)


w_g = calculate_w_g(w, rho_l, g, k_i, k_d, C, d)


# 2.8. Ветровая нагрузка на кабель при гололеде.
# w_{в}=\alpha_{w}*k_{l}*k_{w}*C_{x}*w_0*(d_{каб}+2*k_{i}*k_{d}*C)*10^{-3}
def calculate_w_v(alpha_w, k_l, k_w, C_x, w_0, d_kab, k_i, k_d, C):
    return alpha_w * k_l * k_w * C_x * w_0 * (d_kab + 2 * k_i * k_d * C) * 1e-3


w_v = calculate_w_v(alpha_w, k_l, k_w, C_x, w_0, d_kab, k_i, k_d, C)


# 2.9. Максимальная нагрузка, действующая на кабель
# w_{max} = \sqrt{w^{2}_{г}+w^{2}_{в}}
def calculate_w_max(w_g, w_v):
    return math.sqrt(w_g ** 2 + w_v ** 2)


w_max = calculate_w_max(w_g, w_v)


# где:
# a = \frac{3*(L^{2}+\frac{h^{2}}{2}-L*L_{нк})}{8}
def calculate_a(L, h, L_nk):
    return (3 * (L ** 2 + (h ** 2) / 2 - L * L_nk)) / 8


a = calculate_a(L, h, L_nk)


# b = \frac{-3*W_{max}*L^{3}*L_{нк}}{64*E_{каб}*S_{каб}}
def calculate_b(w_max, L, L_nk, E_kab, s_kab):
    return (-3 * w_max * L ** 3 * L_nk) / (64 * E_kab * s_kab)


b = calculate_b(w_max, L, L_nk, E_kab, s_kab)


def calculate_s_max(a, b):
    # (\frac{a}{3}) ^ {3} + (\frac{-b}{2}) ^ {2}\ge 0
    if (a / 3) ** 3 + (-b / 2) ** 2 >= 0:
        # S_{max}=\sqrt[3]{(\frac{-b}{2})+\sqrt{(\frac{a}{3})^{3}+(\frac{-b}{2})^{2}}}+\sqrt[3]{(\frac{-b}{2})-\sqrt{(\frac{a}{3})^{3}+(\frac{-b}{2})^{2}}}
        term1 = (-b / 2) + math.sqrt((a / 3) ** 3 + (-b / 2) ** 2)
        term2 = (-b / 2) - math.sqrt((a / 3) ** 3 + (-b / 2) ** 2)

        s_max = (term1 ** (1 / 3)) + (term2 ** (1 / 3))
        # s_max = ((-b / 2) + math.sqrt((a / 3)**3 + (-b / 2)**2)**(1/3)) + ((-b / 2) - math.sqrt((a / 3)**3 + (-b / 2)**2)**(1/3))
    else:
        # S_{max}=2*\sqrt{\frac{-a}{3}}*\cos*\left\{ (\frac{1}{3})*\cos^{-1}*\left[ \frac{(\frac{-b}{2})}{(\frac{-a}{3})^{\frac{3}{2}}} \right] \right\}

        term1 = 2 * math.sqrt(-a / 3)
        term2 = (1 / 3) * math.acos(((-b / 2) / ((-a / 3) ** (3 / 2))))

        s_max = term1 * math.cos(term2)
        # s_max = 2 * math.sqrt(-a / 3) * math.cos((1 / 3) * math.acos(((-b / 2) / ((-a / 3) ** (3 / 2)))))
    return s_max


s_max = calculate_s_max(a, b)


# 2.11. Максимальная растягивающая нагрузка при наихудших условиях.
# H_{max}=\frac{W_{max}*L^{2}}{8*S_{max}}
def calculate_s_max(w_max, L, s_max):
    return (w_max * L ** 2) / (8 * s_max)


h_max = calculate_s_max(w_max, L, s_max)

print("1.2. Сечение кабеля - " + str(s_kab))
print("2.1. Вес кабеля - " + str(w_kab))
print("2.2. Растягивающая нагрузка, действующая на кабель - " + str(H))
print("2.3. Перепад высот между опорами. L1 - " + str(L1))
print("2.3. Перепад высот между опорами. L2 - " + str(L2))
print("2.3. Стрелы провиса. S1 - " + str(S1))
print("2.3. Стрелы провиса. S2 - " + str(S2))
print("2.4. Длина подвешенного кабеля - " + str(L_kab))
print("2.5. Длина кабеля в ненагруженном состоянии - " + str(L_no))
print("2.6. Длина кабеля в ненагруженном состоянии с учетом температуры - " + str(L_nk))
print("2.7. Вес кабеля при воздействии максимального гололеда, Wг - " + str(w_g))
print("2.8. Ветровая нагрузка на кабель при гололеде, Wв - " + str(w_v))
print("2.9. Максимальная нагрузка, действующая на кабель, Wмакс - " + str(w_max))
print("2.10. a - " + str(a))
print("2.10. b - " + str(b))
print("2.10. s_max - " + str(s_max))
print("2.11. Максимальная растягивающая нагрузка при наихудших условиях, Hмакс - " + str(h_max))

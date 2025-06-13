import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString, MultiLineString

# Créer le polygone principal
main_polygon = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])

# Créer les polygones d'exclusion
hole1 = Polygon([(2, 2), (3, 2), (3, 3), (2, 3)])
hole2 = Polygon([(7, 7), (8, 7), (8, 8), (7, 8)])

# Soustraire les exclusions du polygone principal
final_polygon = main_polygon.difference(hole1).difference(hole2)

# Calculer le nombre de bandes nécessaires pour couvrir le polygone avec une largeur de 10 cm
band_width = 0.1
num_bands = int(10 / band_width)

# Créer les lignes pour le parcours de la tondeuse
lines = []
for i in range(num_bands):
    x = i * band_width
    line = LineString([(x, 0), (x, 10)]) if i % 2 == 0 else LineString([(x, 10), (x, 0)])
    lines.append(line)

# Créer un MultiLineString pour représenter toutes les lignes
lines = MultiLineString(lines)

# Intersecter les lignes avec le polygone final pour obtenir les segments à tondre
mowing_lines = lines.intersection(final_polygon)

# Tracer le polygone final et les lignes de tonte
x, y = final_polygon.exterior.coords.xy
plt.fill(x, y, color='lightblue', label='Zone de tonte')

for hole in final_polygon.interiors:
    hx, hy = hole.coords.xy
    plt.fill(hx, hy, color='white', label='Exclusion')

# Tracer les lignes de tonte
if mowing_lines.is_empty:
    print("Aucune ligne à tracer.")
elif mowing_lines.geom_type == 'MultiLineString':
    for line in mowing_lines.geoms:  # Utilisez `geoms` pour itérer sur les composants
        x, y = line.xy
        plt.plot(x, y, color='green', label='Trajet de la tondeuse')
elif mowing_lines.geom_type == 'LineString':
    x, y = mowing_lines.xy
    plt.plot(x, y, color='green', label='Trajet de la tondeuse')
else:
    print(f"Type inattendu : {mowing_lines.geom_type}")

# 
# plt.legend()
plt.show()
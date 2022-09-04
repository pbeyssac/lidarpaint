LiDarPaint
==========

### Qu'est ce que c'est ?

Ces scripts colorisent automatiquement des dalles Lidar HD IGN quelconques à partir des données d'orthophotographie distribuées également par l'IGN.

Le script principal `painturl` récupère les fichiers IGN .laz directement à partir de leur URL et s'occupe de tous les traitements, produisant des fichiers color*.laz directement chargeables dans le logiciel displaz.

L'autre script `paintfile.py` travaille avec des fichiers .laz IGN déjà téléchargés et extraits qu'on lui fournit.

Les URL des fichiers .laz sont à trouver sur :
https://geoservices.ign.fr/lidarhd#telechargement

### Comment ça marche :

Le script récupère les données 3D IGN, identifie la position géographique et va alors chercher les dalles 256x256 pixels d'orthophotographie aérienne correspondantes.

Il les assemble ensuite en une image de 1000 x 1000 mètres + marges en coordonnées d'origine de l'API IGN (Web Mercator), puis la convertit en coordonnées Lambert93 utilisées par les dalles Lidar.

Il projette enfin l'image 2D sur l'image Lidar 3D, produisant le fichier final `.laz` utilisable.

### Configuration initiale

La seule configuration indispensable est d'adapter les variables suivantes de `paintfile.py` pour les chemines des logiciels externes :

* `pdal_path`
* `gdalwarp_path`
* `gdal_translate_path`


### Exemple d'utilisation

Pour découvrir Monistrol d'Allier en 3D :

`./painturl 'https://wxs.ign.fr/c90xknypoz1flvgojchbphgt/telechargement/prepackage/LIDARHD_PACK_MN_2021$LIDARHD_1-0_LAZ_MN-0750_6431-2021/file/LIDARHD_1-0_LAZ_MN-0750_6431-2021.7z'`
`displaz *.laz`

Code en licence BSD.

Pierre Beyssac -- 2022
Twitter: @pbeyssac

Les scripts ont été écrits pour Unix mais pourraient fonctionner sur d'autres systèmes, "sur un malentendu".

Les erreurs `proj_create_from_database: crs not found` ou
`(pdal pipeline readers.las Error) GDAL failure (1) PROJ: proj_create_from_database: crs not found`
semblent bénignes (?) et n'empêchent pas le calcul.

Logiciels nécessaires :
* GDAL
* PDAL
* wget
* 7zz
* Python >= 3.9

Modules Python :
* requests
* pyproj
* Pillow (PIL)

Pour l'affichage :
* `displaz`, `CloudCompare`, ...

Displaz doit être configuré avec :
* Colour Mode = color
* Exposure = de 250 à 350 suivant le cas
* et Point Radius = 0,20 donne un résultat plus lisse en vue rapprochée

import setuptools
from wdrone import __version__

long_description = """
	Working project: Optimize the drone flight profile under winds
"""

setuptools.setup(
	name="wdrone",
	version=__version__,
	author="Lan Peng",
	author_email="lanpeng@shu.edu.cn",
	description="wdrone: fly drones under winds",
	long_description=long_description,
	long_description_content_type="text/markdown",
	url="https://github.com/isaac0821/wdrone",
	license='MIT', 
	packages=['wdrone'], 
	download_url = "https://github.com/isaac0821/wdrone",
	project_urls={
		"Bug Tracker": "https://github.com/isaac0821/wdrone/issues",
		"Source Code": "https://github.com/isaac0821/wdrone",
	},
	python_requires='>=3',
	install_requires=[
		'numpy',  
		'scipy',
		'geopy',
		'geojson',
		'matplotlib',
		'shapely',
		'networkx'
	],
	classifiers=[
		"Development Status :: 2 - Pre-Alpha",
		"Programming Language :: Python :: 3",
		"License :: OSI Approved :: MIT License",
		"Operating System :: OS Independent",
	]
)
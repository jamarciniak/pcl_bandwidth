from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Lista dostępnych parametrów 'mode'
    modes = ['raw', 'draco', 'zlib', 'zstd']
    # Wspólny parametr 'dataset_name'
    dataset_name = 'road'

    # Deklaracja argumentów uruchomieniowych
    mode_arg = DeclareLaunchArgument('mode', default_value='default', description='Mode parameter')
    dataset_name_arg = DeclareLaunchArgument('dataset_name', default_value='campus', description='Dataset name parameter')

    # Lista węzłów do uruchomienia
    nodes = []

    # Dla każdego parametru 'mode' tworzymy węzeł i dodajemy go do listy węzłów
    for mode in modes:
        node = Node(
            package='pcl_point_counter',  # Nazwa twojej paczki
            executable='bandwidth_listener',  # Nazwa twojego pliku wykonywalnego
            name=f'bandwidth_subscriber_{mode}',
            parameters=[
                {'mode': mode},
                {'dataset_name': dataset_name}
            ],
            output='screen',
        )
        nodes.append(node)

    # Zwracamy opis uruchomienia zawierający wszystkie węzły
    return LaunchDescription([
        mode_arg,
        dataset_name_arg,
        *nodes  # Rozpakowanie listy węzłów
    ])

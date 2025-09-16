import rclpy

def main():
    rclpy.init()
    # Creiamo un nodo temporaneo solo per poter ispezionare il network
    node = rclpy.create_node('test_node')
    
    print("--- VERIFICA AMBIENTE ROS ---")
    
    # Chiediamo al middleware quali nodi sono visibili a QUESTO script
    node_names = node.get_node_names_and_namespaces()
    print("\nNodi visibili da questo script:")
    if not node_names:
        print(">>> NESSUN NODO TROVATO!")
    else:
        for name, namespace in node_names:
            print(f">>> {namespace}{name}")

    # Chiediamo quali action sono visibili a QUESTO script
    action_names = node.get_action_names_and_types()
    print("\nAction visibili da questo script:")
    if not action_names:
        print(">>> NESSUNA ACTION TROVATA!")
    else:
         for name, types in action_names:
            print(f">>> {name}")

    print("\n--- FINE VERIFICA ---")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

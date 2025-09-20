import asyncio
import sys
import os
import threading
import yaml  

# ciao ciao
# Aggiunge la cartella superiore al path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from textual.app import App, ComposeResult
from textual.widgets import Tabs, Tab, Static, Button, Input, Checkbox

import rclpy
from rclpy.executors import MultiThreadedExecutor
from terminal_node import TerminalNode


class TerminalGUI(App):
    CSS_PATH = "style.css"

    # Quando premi 'q', esegui l'azione 'quit'
    BINDINGS = [("q", "quit", "Quit")]

    def __init__(self):
        super().__init__()

        rclpy.init()
        # per avere il nodo in background
        self.node = TerminalNode()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        
    

        # Carichiamo i parametri qui per averli disponibili in compose()
        params_file = "/home/neo/workspace/src/terminal_gui/params/params.yaml"
        with open(params_file, 'r') as f:
            self.params = yaml.safe_load(f)

    def compose(self) -> ComposeResult:
        # Tabs
        yield Tabs(
            Tab("Services"),
            Tab("Arm/Disarm"),
            Tab("Flight"),
        )

        # Containers
        yield Static("Services Tab", id="services_container")
        yield Static("Arm/Disarm Tab", id="arm_container")
        yield Static("Flight Tab", id="flight_container")

        # --- Services ---
        yield Button("Enable Service", id="enable_service_button")
        yield Button("Reset Service", id="reset_service_button")
        yield Static("Feedback: ", id="services_feedback")

        # --- Arm/Disarm ---
        yield Input(placeholder="Nome Robot", id="arm_name_input")
        yield Button("Arm", id="arm_button")
        yield Button("Disarm", id="disarm_button")
        yield Static("Feedback: ", id="arm_feedback")

        # --- Flight ---
        yield Input(placeholder="Altitudine Takeoff", id="takeoff_input")
        yield Button("Takeoff", id="takeoff_button")
        yield Button("Landing", id="landing_button")

        yield Input(placeholder="X Navigate", id="navigate_x")
        yield Input(placeholder="Y Navigate", id="navigate_y")
        yield Input(placeholder="Z Navigate", id="navigate_z")
        yield Button("Navigate", id="navigate_button")

        # widget con i valori di default corretti
        yield Static("Feedback: ", id="flight_feedback")

    def on_mount(self) -> None:
        """
        Chiamato quando l'app è pronta.
        ORA inizializziamo il nodo ROS e avviamo lo spin.
        """
        #ROBMASOCCO
        #self.ros_thread = Thread(target=self.executor.spin, daemon=True)
        #self.ros_thread.start()

        self.ros_executor = MultiThreadedExecutor()
        self.node = TerminalNode()  # <-- Il nodo viene creato ADESSO
        self.ros_executor.add_node(self.node)
        
        # Crea e avvia il thread per lo spin di ROS in background
        self.ros_thread = threading.Thread(target=self.ros_executor.spin, daemon=True)
        self.ros_thread.start() 
        
        self.set_tab_visibility("Services")

    def shutdown_ros(self):
        """Metodo centralizzato per chiudere ROS in modo pulito."""
        print("Inizio shutdown di ROS...")
        if self.ros_executor:
            self.ros_executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print("Terminando la GUI...")

    def on_unmount(self) -> None:
        """Chiamato quando l'app si chiude per qualsiasi motivo."""
        #self.shutdown_ros()
        #ROB
        self.executor.shutdown()
        rclpy.shutdown()

    def action_quit(self) -> None:
        """Chiamato quando viene premuto il tasto 'q'."""
        self.exit()

    def set_tab_visibility(self, tab_name: str):
        self.query_one("#services_container").display = (tab_name == "Services")
        self.query_one("#arm_container").display = (tab_name == "Arm/Disarm")
        self.query_one("#flight_container").display = (tab_name == "Flight")

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = event.button.id
        try:
            # --- Services ---
            if button_id == "enable_service_button":
                self.query_one("#services_feedback", Static).update("Chiamata a Enable Service in corso...")
                result = await self.node.call_enable_service()
                self.query_one("#services_feedback", Static).update(f"Enable Service riuscito: {result.message}")
            elif button_id == "reset_service_button":
                self.query_one("#services_feedback", Static).update("Chiamata a Reset Service in corso...")
                result = await self.node.call_reset_service()
                self.query_one("#services_feedback", Static).update(f"Reset Service riuscito: {result.message}")
            # --- Arm/Disarm ---
            elif button_id == "arm_button":
                name = self.query_one("#arm_name_input", Input).value
                if not name.strip():
                    self.query_one("#arm_feedback", Static).update("Errore: inserire un nome valido")
                    return
                self.query_one("#arm_feedback", Static).update(f"Invio richiesta ARM per '{name}'...")
                result = await self.node.send_arm_goal(name)
                self.query_one("#arm_feedback", Static).update(f"Feedback ARM: {result.status}")
            elif button_id == "disarm_button":
                name = self.query_one("#arm_name_input", Input).value
                if not name.strip():
                    self.query_one("#arm_feedback", Static).update("Errore: inserire un nome valido")
                    return
                self.query_one("#arm_feedback", Static).update(f"Invio richiesta DISARM per '{name}'...")
                result = await self.node.send_disarm_goal(name)
                self.query_one("#arm_feedback", Static).update(f"Feedback DISARM: {result.status}")
            # --- Flight ---
            elif button_id == "takeoff_button":
                input_value = self.query_one("#takeoff_input", Input).value
                try: altitude = float(input_value)
                except ValueError:
                    self.query_one("#flight_feedback", Static).update("Errore: Altitudine non valida")
                    return
                self.query_one("#flight_feedback", Static).update(f"Takeoff a {altitude}m in corso...")
                result = await self.node.send_takeoff_goal(altitude)
                self.query_one("#flight_feedback", Static).update(f"Takeoff completato con successo")
            elif button_id == "landing_button":
                self.query_one("#flight_feedback", Static).update("Landing in corso...")
                result = await self.node.send_landing_goal()
                self.query_one("#flight_feedback", Static).update(f"Landing completato con successo")
            elif button_id == "navigate_button":
                try:
                    x = float(self.query_one("#navigate_x", Input).value)
                    y = float(self.query_one("#navigate_y", Input).value)
                    z = float(self.query_one("#navigate_z", Input).value)
                except ValueError:
                    self.query_one("#flight_feedback", Static).update("Errore: coordinate non valide")
                    return
                self.query_one("#flight_feedback", Static).update(f"Navigazione verso ({x}, {y}, {z}) in corso...")
                result = await self.node.send_navigate_goal(x, y, z)
                self.query_one("#flight_feedback", Static).update(f"Navigazione completata con successo")
        except Exception as e:
            if button_id in ["enable_service_button", "reset_service_button"]:
                self.query_one("#services_feedback", Static).update(f"ERRORE: {e}")
            elif button_id in ["arm_button", "disarm_button"]:
                self.query_one("#arm_feedback", Static).update(f"ERRORE: {e}")
            else:
                self.query_one("#flight_feedback", Static).update(f"ERRORE: {e}")

if __name__ == "__main__":
    app = TerminalGUI()
    app.run()
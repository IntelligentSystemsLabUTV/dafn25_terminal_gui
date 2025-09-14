import asyncio
import sys
import os
import threading

# Aggiunge la cartella superiore al path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from textual.app import App, ComposeResult
from textual.widgets import Tabs, Tab, Static, Button, Input, Checkbox

import rclpy
from rclpy.executors import MultiThreadedExecutor
from terminal_node import TerminalNode


class TerminalGUI(App):
    CSS_PATH = "style.css"

    # Quando l'utente preme 'q', esegui l'azione 'quit'
    BINDINGS = [("q", "quit", "Quit")]

    def __init__(self):
        super().__init__()
        rclpy.init()
        
        # Usa un MultiThreadedExecutor per eseguire lo spin di ROS in background
        self.ros_executor = MultiThreadedExecutor()
        self.node = TerminalNode()
        self.ros_executor.add_node(self.node)
        
        # Crea il thread per lo spin, ma non avviarlo ancora
        self.ros_thread = threading.Thread(target=self.ros_executor.spin, daemon=True)

    def compose(self) -> ComposeResult:
        # Il tuo metodo compose rimane invariato...
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

        yield Input(value=str(self.node.params['defaults']['throttle']),
                    placeholder="Throttle (0-1)", id="throttle_input")
        yield Checkbox(label="Safety Check", value=self.node.params['defaults']['safety_check'],
                       id="safety_checkbox")
        yield Input(value=self.node.params['defaults']['flight_mode'],
                    placeholder="Flight Mode", id="flight_mode_input")
        yield Static("Feedback: ", id="flight_feedback")


    def on_mount(self) -> None:
        """Chiamato quando l'app è pronta."""
        self.ros_thread.start() # Avvia lo spin di ROS in background
        self.set_tab_visibility("Services")

    def on_unmount(self) -> None:
        """Chiamato quando l'app si chiude."""
        self.ros_executor.shutdown()
        rclpy.shutdown()

    def set_tab_visibility(self, tab_name: str):
        self.query_one("#services_container").display = (tab_name == "Services")
        self.query_one("#arm_container").display = (tab_name == "Arm/Disarm")
        self.query_one("#flight_container").display = (tab_name == "Flight")

    def on_exit(self) -> None:
        """Chiamato quando Textual sta per terminare."""
        # Ferma l'executor ROS (se è stato avviato)
        if hasattr(self, 'ros_executor'):
            self.ros_executor.shutdown()
            rclpy.shutdown()

        # Aggiungi un messaggio di log (opzionale)
        print("Terminando la GUI...")

    def action_quit(self) -> None:
        """Chiamato quando viene premuto il tasto associato all'azione 'quit'."""
        self.exit() # Chiude l'applicazione in modo pulito

    async def on_button_pressed(self, event: Button.Pressed) -> None:
        try:
            # --- Services ---
            if event.button.id == "enable_service_button":
                self.query_one("#services_feedback", Static).update("Chiamata a Enable Service in corso...")
                result = await self.node.call_enable_service()
                self.query_one("#services_feedback", Static).update(f"Enable Service riuscito: {result.message}")

            elif event.button.id == "reset_service_button":
                self.query_one("#services_feedback", Static).update("Chiamata a Reset Service in corso...")
                result = await self.node.call_reset_service()
                self.query_one("#services_feedback", Static).update(f"Reset Service riuscito: {result.message}")

            # --- Arm/Disarm ---
            elif event.button.id == "arm_button":
                name = self.query_one("#arm_name_input", Input).value
                if not name.strip():
                    self.query_one("#arm_feedback", Static).update("Errore: inserire un nome valido")
                    return
                self.query_one("#arm_feedback", Static).update(f"Invio richiesta ARM per '{name}'...")
                result = await self.node.send_arm_goal(name)
                self.query_one("#arm_feedback", Static).update(f"Feedback ARM: {result.status}")

            elif event.button.id == "disarm_button":
                name = self.query_one("#arm_name_input", Input).value
                if not name.strip():
                    self.query_one("#arm_feedback", Static).update("Errore: inserire un nome valido")
                    return
                self.query_one("#arm_feedback", Static).update(f"Invio richiesta DISARM per '{name}'...")
                result = await self.node.send_disarm_goal(name)
                self.query_one("#arm_feedback", Static).update(f"Feedback DISARM: {result.status}")

            # --- Flight ---
            elif event.button.id == "takeoff_button":
                input_value = self.query_one("#takeoff_input", Input).value
                try:
                    altitude = float(input_value)
                    if not (0 <= altitude <= 100):
                        raise ValueError("Altitudine fuori range (0-100)")
                except ValueError as e:
                    self.query_one("#flight_feedback", Static).update(f"Errore: {e}")
                    return
                self.query_one("#flight_feedback", Static).update(f"Takeoff a {altitude}m in corso...")
                result = await self.node.send_takeoff_goal(altitude)
                self.query_one("#flight_feedback", Static).update(f"Takeoff completato: {result.success}")

            elif event.button.id == "landing_button":
                self.query_one("#flight_feedback", Static).update("Landing in corso...")
                result = await self.node.send_landing_goal()
                self.query_one("#flight_feedback", Static).update(f"Landing completato: {result.success}")

            elif event.button.id == "navigate_button":
                try:
                    x = float(self.query_one("#navigate_x", Input).value)
                    y = float(self.query_one("#navigate_y", Input).value)
                    z = float(self.query_one("#navigate_z", Input).value)
                except ValueError:
                    self.query_one("#flight_feedback", Static).update("Errore: coordinate non valide")
                    return
                self.query_one("#flight_feedback", Static).update(f"Navigazione verso ({x}, {y}, {z}) in corso...")
                result = await self.node.send_navigate_goal(x, y, z)
                self.query_one("#flight_feedback", Static).update(f"Navigazione completata: {result.success}")

        except Exception as e:
            # Cattura errori di comunicazione ROS (timeout, etc.) e mostrali nella GUI
            # Questa è una gestione generica, puoi migliorarla per aggiornare il campo corretto
            self.query_one("#arm_feedback", Static).update(f"ERRORE: {e}")
            self.query_one("#flight_feedback", Static).update(f"ERRORE: {e}")
            self.query_one("#services_feedback", Static).update(f"ERRORE: {e}")

if __name__ == "__main__":
    TerminalGUI().run()
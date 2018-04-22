import hlt
import logging
import math
from math import pi
import random

# GAME START
# Here we define the bot's name as Settler and initialize the game, including communication with the Halite engine.
game = hlt.Game("PlayBot")
turn = 0
initial_ships = []
first_turn_planets = {}
first_turn_enemy_ships = {}

def get_closest_planets(ship, reverse=False, enemy_only=False):
    distances = []
    all_planets = game_map.all_planets()

    if enemy_only:
        all_planets =  [ x for x in all_planets if x.owner and x.owner.id != game_map.my_id ]
        #logging.info("ENEMY ONLY: My id: {} | Enemy only planets: {}".format(game_map.my_id, all_planets))

    for planet in all_planets:
        distances.append((planet, ship.calculate_distance_between(planet)))

    distances = sorted(distances, key=lambda x: x[1], reverse=reverse)

    distances = list(map(lambda x: x[0], distances))
    # logging.info(distances)
    return distances

def get_total_ship_count():
    total = 0
    for player in game_map.all_players():
        total += len(player.all_ships())

    return total

def furthest_planets_from_opponent():
    all_players = game_map.all_players()
    opponents = filter(lambda x: x.id != game_map.my_id, all_players)
    opponent_homes = []
    for opponent in opponents:
        opponent_ships = opponent.all_ships()
        ship_avg = (0.0, 0.0)
        for ship in opponent_ships:
            ship_avg = (((ship_avg[0] + ship.x) / 2, (ship_avg[1] + ship.y) / 2))
        opponent_homes.append((opponent.id, ship_avg))

    return opponent_homes

def get_opponent_strengths():
    opponent_strengths = []
    opponents = [x.id for x in game_map.all_players()]
    ships = game_map._all_ships()
    for player in opponents:
        opponent_strengths.append(
            (
                player,
                len([x for x in ships if x.owner.id == player]),
                len([x for x in ships if x.docking_status != x.DockingStatus.UNDOCKED])
            )
        )

    return opponent_strengths

def get_closest_enemy_ships(my_ship, opponent_id, reverse=False, include_distance=False):
    opponent = game_map.get_player(opponent_id)
    enemy_ships = opponent.all_ships()
    distances = []

    for ship in enemy_ships:
        distances.append((ship, my_ship.calculate_distance_between(ship)))

    distances = sorted(distances, key=lambda x: x[1], reverse=reverse)
    if not include_distance:
        distances = list(map(lambda x: x[0], distances))
    logging.info(distances)
    return distances

def points_on_planet(center=(0.0, 0.0), r=3, n=100, ship_buffer=0.5):
    return [
        (
                round(center[0] + (math.cos(2*pi/n*x) * (r + ship_buffer)), 4),
                round(center[1] + (math.sin(2*pi/n*x) * (r + ship_buffer)), 4)
        ) for x in range(0, n+1) ]

def get_closest_obstacle(ship, obstacles):
    distances = []
    for planet in obstacles:
        distances.append((planet, ship.calculate_distance_between(planet)))

    distances = sorted(distances, key=lambda x: x[1])
    obstacle = distances[0][0]

    return obstacle

def find_bypass_vertices(ship, obstacle, destination, obstacle_buffer=2):
    # obstacle_buffer == 1.5 because ship diamter = 1, and
    # to pass without hitting a docked ship is 1.5 out
    # For each of the obstacles we want to add two nodes
    # perpendicular to the collision line, and two edges
    # to these nodes to create a path
    vertices = []
    # # logging.info("Parsing Obstacle: {}".format(obstacle))

    if ship.x - obstacle.x == 0:
        slope_to_obstacle = 1
    elif ship.y - obstacle.y == 0:
        slope_to_obstacle = 0.000001
    else:
        slope_to_obstacle = (ship.y - obstacle.y) / (ship.x - obstacle.x)

    slope_to_vertice = -1/slope_to_obstacle
    r = math.sqrt(1 + slope_to_vertice**2)

    vertices.append(
        hlt.entity.Position(
            obstacle.x + ((obstacle.radius + obstacle_buffer) / r),
            obstacle.y + (( (obstacle.radius + obstacle_buffer) * slope_to_vertice) / r)
        )
    )
    vertices.append(
        hlt.entity.Position(
            obstacle.x - ((obstacle.radius + obstacle_buffer) / r),
            obstacle.y - (( (obstacle.radius + obstacle_buffer) * slope_to_vertice) / r)
        )
    )

    return vertices

def nav(ship, destination, ob_buf=random.uniform(1, 2.5)):
    obstacles = game_map.obstacles_between(ship, destination)
    speed = hlt.constants.MAX_SPEED
    command = None
    # logging.info("Found obstacles! Between ship: {} and obstacles: {}".format(ship, obstacles))

    if obstacles:
        obstacle = get_closest_obstacle(ship, obstacles)

        # If the closest obstacle is still far away, get closer to avoid making more obstacles
        if ship.calculate_distance_between(obstacle) - (obstacle.radius) > 13:
            angle = ship.calculate_angle_between(destination)
        else:
            clear_path = False
            ob_buf = 2
            try_count = 0
            while not clear_path and try_count < 4:
                vertices = find_bypass_vertices(ship, obstacle, destination, obstacle_buffer=ob_buf)
                for vertice in vertices:
                    if not game_map.obstacles_between(ship, vertice):
                        clear_path = True
                        break
                    else:
                        try_count += 1
                        ob_buf += 1

            # Can't find a clear path
            if try_count > 3:
                command = ship.navigate(
                    ship.closest_point_to(destination,min_distance=3),
                    game_map, speed=speed,angular_step=10
                )
            bypass = vertice
            angle = ship.calculate_angle_between(bypass)
    else:
        angle = ship.calculate_angle_between(destination)
        distance = ship.calculate_distance_between(destination) - (destination.radius + ship.radius)
        if distance < 8:
            speed = distance
            return ship.navigate(
                ship.closest_point_to(destination,min_distance=3),
                game_map, speed=speed, angular_step=10, max_corrections=180
                )

    # Determine speed based on surroundings
    # nearby = game_map.nearby_entities_by_distance(ship)
    # very_close = []

    # for distance, entities in nearby.items():
    #     if distance < 10:
    #         very_close += entities

    if not command:
        command = ship.thrust(speed, angle)

    return command

def closest_unprotected_planet(ship):
    closest_planets = get_closest_planets(ship, enemy_only=True)

    if closest_planets:
        for planet in closest_planets:
            nearby_entities = game_map.nearby_entities_by_distance(planet)

            nearby_entities = [ value for key,value in nearby_entities.items() if key < planet.radius + 4 ]
            logging.info(nearby_entities)

            if len(nearby_entities) == len(planet.all_docked_ships()):
                logging.info("FOUND UNPROTECTED PLANET: {}".format(planet))
                return planet

    return None

while True:
    # TURN START
    # Update the map for the new turn and get the latest version
    game_map = game.update_map()
    logging.info("TURN: {}".format(turn))
    command_queue = []

    opponent_strengths = get_opponent_strengths()
    logging.info("opponent_strengths: {}".format(opponent_strengths))
    all_ships = game_map.get_me().all_ships()
    if turn == 0:
        for ship in all_ships:
            initial_ships.append(ship.id)

        opponent_id = [x for x in opponent_strengths if x[0] != game_map.my_id][0][0]
        enemy_ships = get_closest_enemy_ships(all_ships[0], opponent_id, include_distance=True)
        too_damn_far = True if enemy_ships[0][1] > 150 else False


    # 4-player Strategy
    # if len(opponent_strengths) > 2 or too_damn_far:
    if True:
        for ship in all_ships:
            navigate_command = None

            # Keep docked ships producing and already commanded ships steadfast
            if ship.docking_status == ship.DockingStatus.DOCKED:
                continue

            if command_queue:
                pass

            logging.info("Initial_ships: {}".format(initial_ships))
            # Spread out initial ships for quick colonization
            if ship.id in initial_ships:
                logging.info("Checking if {} is in first turn values".format(ship.id))
                if ship.id in first_turn_planets.values() and turn > 0:
                    logging.info("{} is already in {}, keeping original heading".format(ship.id, first_turn_planets))
                    planet_id = list(first_turn_planets.keys())[list(first_turn_planets.values()).index(ship.id)]
                    planet = game_map.get_planet(planet_id)

                    if ship.can_dock(planet):
                        logging.info("{} is docking to {}".format(ship.id, planet.id))
                        navigate_command = ship.dock(planet)
                    else:
                        logging.info("{} is heading to {}".format(ship.id, planet.id))
                        navigate_command = nav(ship, planet)

                    if navigate_command:
                        command_queue.append(navigate_command)
                        continue
                else:
                    closest_planets = get_closest_planets(ship)
                    for planet in closest_planets:
                        logging.info("Finding closest ship for closest planet: {}".format(planet))
                        if planet.id not in first_turn_planets.keys():
                            logging.info("{} is not in {}".format(planet.id, first_turn_planets))

                            distance_to_planet = []
                            for ship in initial_ships:
                                ship = game_map.get_me().get_ship(ship)
                                distance_to_planet.append((ship, ship.calculate_distance_between(planet)))

                            distance_to_planet = sorted(distance_to_planet, key=lambda x:x[1])
                            distance_to_planet = [x for x in distance_to_planet if x[0].id not in first_turn_planets.values()]
                            logging.info("Closest planets by distance: {}".format(distance_to_planet))
                            ship = distance_to_planet[0][0]

                            first_turn_planets[planet.id] = ship.id
                            if ship.can_dock(planet):
                                logging.info("{} is docking to {}".format(ship.id, planet.id))
                                navigate_command = ship.dock(planet)
                            else:
                                logging.info("{} is heading to {}".format(ship.id, planet.id))
                                navigate_command = nav(ship, planet)

                            if navigate_command:
                                command_queue.append(navigate_command)
                                break



                    if navigate_command:
                        continue

            # If we have sufficiently high production we should attack
            # unprotected planets
            production_ships = [x for x in all_ships if x.docking_status != ship.DockingStatus.UNDOCKED]
            highest_opponent_strength = sorted(opponent_strengths, key=lambda x:x[2], reverse=True)[0][2]
            if len(production_ships) > (3/4 * highest_opponent_strength):
                planet = closest_unprotected_planet(ship)
                if planet:
                    docked_ships = list(planet._docked_ships.values())
                    logging.info("{} is attacking at planet {}".format(ship.id, planet.id))
                    navigate_command = nav(ship, docked_ships[0])
                    if navigate_command:
                        command_queue.append(navigate_command)
                        continue

            # Check if all planets are occupied or we aren't strong enough to determine farm or attack
            all_planets = game_map.all_planets()
            open_planets = {x for x in all_planets if x.owner == None}
            if len(open_planets) > 0:
                closest_planets = get_closest_planets(ship)

                #Start claiming closest unowned planets
                for planet in closest_planets:
                    # planet = planet[0]
                    if len(planet.all_docked_ships()) < planet.num_docking_spots and (planet.owner is None or planet.owner == game_map.get_me()):
                        if ship.can_dock(planet):
                            logging.info("{} is docking at open planet {}".format(ship.id, planet.id))
                            command_queue.append(ship.dock(planet))
                        else:
                            logging.info("{} is heading to open planet {}".format(ship.id, planet.id))
                            navigate_command = nav(ship, planet)
                            if navigate_command:
                                command_queue.append(navigate_command)
                        break
            else:
                # opponent_strengths = get_opponenet_strengths()
                closest_planets = get_closest_planets(ship, enemy_only=True)
                for planet in closest_planets:
                    docked_ships = list(planet._docked_ships.values())
                    logging.info("{} is going after a docked ship at planet {}".format(ship.id, planet.id))
                    if docked_ships:
                        if get_total_ship_count() > 100:
                            navigate_command = nav(ship, docked_ships[0])
                        else:
                            navigate_command = ship.navigate(
                                ship.closest_point_to(docked_ships[0],min_distance=3),
                                game_map, speed=7, angular_step=15, max_corrections=210
                            )
                    if navigate_command:
                        command_queue.append(navigate_command)
                        break

            if not navigate_command:
                logging.info("{} is DOING ABSOLUTELY NOTHING".format(ship.id))

    # 2-player Strategy
    else:
        for ship in all_ships:
            navigate_command = None

            # Keep docked ships producing and already commanded ships steadfast
            if ship.docking_status == ship.DockingStatus.DOCKED:
                continue

            if command_queue:
                pass

            logging.info("Initial_ships: {}".format(initial_ships))
            # Send initial ships for the throat
            if ship.id in initial_ships:
                logging.info("Checking if {} is in first turn values".format(ship.id))
                if ship.id in first_turn_enemy_ships.values() and turn > 0:
                    logging.info("{} is already in {}, keeping original heading".format(ship.id, first_turn_enemy_ships))
                    enemy_ship_id = list(first_turn_enemy_ships.keys())[list(first_turn_enemy_ships.values()).index(ship.id)]
                    enemy_ship = game_map.get_player(opponent_id).get_ship(enemy_ship_id)

                    if enemy_ship:
                        logging.info("{} is hunting down {}".format(ship.id, enemy_ship.id))
                        # navigate_command = nav(ship, enemy_ship)
                        navigate_command = ship.navigate(
                            ship.closest_point_to(enemy_ship,min_distance=1),
                            game_map, speed=7, angular_step=15, max_corrections=210
                        )

                        if navigate_command:
                            command_queue.append(navigate_command)
                            continue
                    else:
                        # Targeted ship is dead, return to pool
                        initial_ships.remove(ship.id)

                else:
                    closest_enemy_ships = get_closest_enemy_ships(ship, opponent_id)
                    for enemy_ship in closest_enemy_ships:
                        logging.info("Finding closest ship for closest enemy_ship: {}".format(enemy_ship))
                        if enemy_ship.id not in first_turn_enemy_ships.keys():
                            logging.info("{} is not in {}".format(enemy_ship.id, first_turn_enemy_ships))

                            distance_to_enemy_ship = []
                            for ship in initial_ships:
                                ship = game_map.get_me().get_ship(ship)
                                distance_to_enemy_ship.append((ship, ship.calculate_distance_between(enemy_ship)))

                            distance_to_enemy_ship = sorted(distance_to_enemy_ship, key=lambda x:x[1])
                            distance_to_enemy_ship = [x for x in distance_to_enemy_ship if x[0].id not in first_turn_enemy_ships.values()]
                            logging.info("Closest enemy_ships by distance: {}".format(distance_to_enemy_ship))
                            ship = distance_to_enemy_ship[0][0]

                            first_turn_enemy_ships[enemy_ship.id] = ship.id

                            logging.info("{} is hunting down {}".format(ship.id, enemy_ship.id))
                            # navigate_command = ship.navigate(
                            #     ship.closest_point_to(enemy_ship,min_distance=1),
                            #     game_map, speed=7 - ship.id / 4, angular_step=15, max_corrections=210
                            # )
                            navigate_command = ship.thrust(7, ship.calculate_angle_between(enemy_ship) + ship.id * 3)

                            if navigate_command:
                                command_queue.append(navigate_command)
                                break



                    if navigate_command:
                        continue

            # If we have sufficiently high production we should attack
            # unprotected planets
            production_ships = [x for x in all_ships if x.docking_status != ship.DockingStatus.UNDOCKED]
            highest_opponent_strength = sorted(opponent_strengths, key=lambda x:x[2], reverse=True)[0][2]
            if len(production_ships) > (3/4 * highest_opponent_strength):
                planet = closest_unprotected_planet(ship)
                if planet:
                    docked_ships = list(planet._docked_ships.values())
                    logging.info("{} is attacking at planet {}".format(ship.id, planet.id))
                    navigate_command = nav(ship, docked_ships[0])
                    if navigate_command:
                        command_queue.append(navigate_command)
                        continue

            # Check if all planets are occupied or we aren't strong enough to determine farm or attack
            all_planets = game_map.all_planets()
            open_planets = {x for x in all_planets if x.owner == None}
            if len(open_planets) > 0:
                closest_planets = get_closest_planets(ship)

                #Start claiming closest unowned planets
                for planet in closest_planets:
                    # planet = planet[0]
                    if len(planet.all_docked_ships()) < planet.num_docking_spots and (planet.owner is None or planet.owner == game_map.get_me()):
                        if ship.can_dock(planet):
                            logging.info("{} is docking at open planet {}".format(ship.id, planet.id))
                            command_queue.append(ship.dock(planet))
                        else:
                            logging.info("{} is heading to open planet {}".format(ship.id, planet.id))
                            navigate_command = nav(ship, planet)
                            if navigate_command:
                                command_queue.append(navigate_command)
                        break
            else:
                # opponent_strengths = get_opponenet_strengths()
                closest_planets = get_closest_planets(ship, enemy_only=True)
                for planet in closest_planets:
                    docked_ships = list(planet._docked_ships.values())
                    logging.info("{} is going after a docked ship at planet {}".format(ship.id, planet.id))
                    if docked_ships:
                        if get_total_ship_count() > 100:
                            navigate_command = nav(ship, docked_ships[0])
                        else:
                            navigate_command = ship.navigate(
                                ship.closest_point_to(docked_ships[0],min_distance=3),
                                game_map, speed=7, angular_step=15, max_corrections=210
                            )
                    if navigate_command:
                        command_queue.append(navigate_command)
                        break

            if not navigate_command:
                logging.info("{} is DOING ABSOLUTELY NOTHING".format(ship.id))


    turn += 1
    logging.info("Current command queue: {}".format(command_queue))
    game.send_command_queue(command_queue)
    # TURN END
# GAME END

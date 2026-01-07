# Guide Groot2 pour TurtleShot BT

## Connexion au Monitoring Live

### ⚠️ Important : Ce n'est PAS WebSocket !

La connexion Groot2 utilise **ZMQ** (pas ws://). Voici la bonne méthode :

### Étapes de connexion

1. **Lancer le BT node**
   ```bash
   ros2 run turtleshot_bt turtleshot_bt_node
   ```

   Vous devriez voir :
   ```
   [INFO] ✓ ZMQ Publisher started (ports: 1666/1667)
   [INFO]   → In Groot2: Monitor → Connect
   [INFO]      Publisher port: 1666
   [INFO]      Server port: 1667
   ```

2. **Ouvrir Groot2**

3. **Aller en mode Monitor**
   - Cliquer sur l'onglet **"Monitor"** en haut

4. **Configurer la connexion**
   - Cliquer sur **"Connect"**
   - Dans la fenêtre de connexion :
     - **Address**: `127.0.0.1` ou `localhost`
     - **Publisher Port**: `1666`
     - **Server Port**: `1667`
   - ❌ **NE PAS** mettre `ws://` ou `http://`
   - ✅ Juste l'adresse IP et les ports

5. **Cliquer sur "Connect"**

Vous devriez voir l'arbre s'afficher avec les nodes colorés en temps réel :
- 🟢 **Vert** = SUCCESS
- 🟡 **Jaune** = RUNNING
- 🔴 **Rouge** = FAILURE
- ⚪ **Gris** = IDLE

## Éditer l'Arbre

### Charger l'arbre pour édition

1. Dans Groot2, onglet **"Editor"**
2. **File → Load Tree**
3. Naviguer vers :
   ```
   install/turtleshot_bt/share/turtleshot_bt/trees/turtleshot_mission.xml
   ```

### Éditer

- **Drag & drop** nodes depuis la palette gauche
- **Double-clic** sur un node pour éditer ses paramètres
- **Clic droit** → Delete pour supprimer
- **Connecter** nodes en glissant depuis les ports

### Sauvegarder

1. **File → Save Tree**
2. Sauvegarder dans le même fichier
3. **Redémarrer** le BT node pour appliquer les changements

## Troubleshooting

### "Cannot connect to server"

**Problème** : Groot2 ne trouve pas le serveur ZMQ

**Solutions** :
1. Vérifier que le BT node est en cours d'exécution
2. Vérifier les ports dans les logs du node
3. Vérifier qu'aucun firewall ne bloque les ports 1666/1667
4. Essayer `127.0.0.1` au lieu de `localhost`

### "Connection refused"

**Problème** : Les ports sont déjà utilisés

**Solutions** :
```bash
# Linux/Mac : Vérifier qui utilise les ports
sudo lsof -i :1666
sudo lsof -i :1667

# Windows
netstat -ano | findstr "1666"
netstat -ano | findstr "1667"
```

### "Tree not updating"

**Problème** : L'arbre ne se met pas à jour en temps réel

**Solutions** :
1. Déconnecter et reconnecter dans Groot2
2. Vérifier que `PublisherZMQ` est bien créé dans le code
3. Redémarrer le BT node

## Modes Groot2

### Editor Mode
- Créer et modifier des arbres
- Sauvegarder en XML
- Pas de connexion au robot nécessaire

### Monitor Mode
- Visualiser l'exécution en temps réel
- Voir les transitions d'état
- Nécessite une connexion ZMQ active

### Replay Mode
- Rejouer des logs d'exécution
- Utilise les fichiers .fbl (FileLogger)

## Exemples de Configuration

### Configuration par défaut
```
Address: 127.0.0.1
Publisher Port: 1666
Server Port: 1667
```

### Configuration distante
Si le robot est sur une autre machine :
```
Address: 192.168.1.100  (IP du robot)
Publisher Port: 1666
Server Port: 1667
```

### Plusieurs robots
Changez les ports dans le code pour chaque robot :
```cpp
BT::PublisherZMQ publisher_zmq(tree, 10, 2666, 2667);  // Robot 2
BT::PublisherZMQ publisher_zmq(tree, 10, 3666, 3667);  // Robot 3
```

## Tests

### Tester la connexion ZMQ

Sans Groot2, vous pouvez tester avec `zmq_sub` :

```bash
# Installer zmq tools
sudo apt install libzmq3-dev

# Écouter le publisher port
zmq_sub tcp://localhost:1666
```

Si vous voyez des données binaires défiler, ZMQ fonctionne !

## Ressources

- [Groot2 Documentation](https://www.behaviortree.dev/groot/)
- [BehaviorTree.CPP ZMQ Logger](https://www.behaviortree.dev/docs/tutorial-advanced/tutorial_11_groot_howto)
- [ZMQ Documentation](https://zeromq.org/)

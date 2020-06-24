import roslibpy

client = roslibpy.Ros(host='127.0.0.1', port=11311)
client.run()

service = roslibpy.Service(client, 'manager_service', 'planner/Manager')

request = {'test'}

print('Calling service...')
result = service.call(request)
print(result.response)

client.terminate()
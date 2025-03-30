#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <optional>
#include <array>

const float GRAVITY = 9.8f;
const float PIXELS_PER_METER = 50.0f;
const float ELASTICITY = 0.8f;

enum class ShapeType {
    Circle,
    Rectangle,
    Triangle
};
struct Projection {
    float min;
    float max;

    bool overlaps(const Projection& other) const {
        return !(max < other.min || other.max < min);
    }
};

class Body {
public:
    ShapeType shapeType;
    sf::CircleShape circleShape;
    sf::RectangleShape rectShape;
    sf::ConvexShape triangleShape;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;
    float mass;
    float frictionCoeff;
    bool isStatic;
    float radius; // Используется для круга
    sf::Vector2f size; // Используется для прямоугольника

    Body(ShapeType type, float radiusOrSize, sf::Vector2f position, sf::Color color,
        float mass = 1.0f, float friction = 0.2f, bool isStatic = false)
        : shapeType(type), mass(mass), frictionCoeff(friction), isStatic(isStatic),
        radius(radiusOrSize), size(radiusOrSize, radiusOrSize) {

        velocity = sf::Vector2f(0, 0);
        acceleration = sf::Vector2f(0, 0);

        switch (shapeType) {
        case ShapeType::Circle:
            circleShape.setRadius(radius);
            circleShape.setPosition(position);
            circleShape.setFillColor(color);
            circleShape.setOrigin({ radius, radius });
            break;

        case ShapeType::Rectangle:
            rectShape.setSize(size);
            rectShape.setPosition(position);
            rectShape.setFillColor(color);
            rectShape.setOrigin(size / 2.0f);
            break;

        case ShapeType::Triangle:
            triangleShape.setPointCount(3);
            triangleShape.setPoint(0, { 0, -radiusOrSize });
            triangleShape.setPoint(1, { radiusOrSize, radiusOrSize });
            triangleShape.setPoint(2, { -radiusOrSize, radiusOrSize });
            triangleShape.setPosition(position);
            triangleShape.setFillColor(color);
            triangleShape.setOrigin({ 0, 0 });
            break;
        }
    }

    void update(float dt) {
        if (isStatic) return;

        acceleration.y += GRAVITY * PIXELS_PER_METER;

        if (getPosition().y + getBoundingBox().size.y / 2 >= 550.f) {
            float frictionForce = frictionCoeff * mass * GRAVITY * PIXELS_PER_METER;
            sf::Vector2f friction(-std::copysign(frictionForce, velocity.x), 0);
            acceleration += friction / mass;
        }

        velocity += acceleration * dt;
        move(velocity * dt);
        acceleration = sf::Vector2f(0, 0);
    }

    void move(sf::Vector2f offset) {
        switch (shapeType) {
        case ShapeType::Circle:
            circleShape.move(offset);
            break;
        case ShapeType::Rectangle:
            rectShape.move(offset);
            break;
        case ShapeType::Triangle:
            triangleShape.move(offset);
            break;
        }
    }

    sf::Vector2f getPosition() const {
        switch (shapeType) {
        case ShapeType::Circle: return circleShape.getPosition();
        case ShapeType::Rectangle: return rectShape.getPosition();
        case ShapeType::Triangle: return triangleShape.getPosition();
        }
        return { 0,0 };
    }

    void setPosition(sf::Vector2f position) {
        switch (shapeType) {
        case ShapeType::Circle:
            circleShape.setPosition(position);
            break;
        case ShapeType::Rectangle:
            rectShape.setPosition(position);
            break;
        case ShapeType::Triangle:
            triangleShape.setPosition(position);
            break;
        }
    }

    sf::FloatRect getBoundingBox() const {
        switch (shapeType) {
        case ShapeType::Circle:
            return circleShape.getGlobalBounds();
        case ShapeType::Rectangle:
            return rectShape.getGlobalBounds();
        case ShapeType::Triangle: {
            // Для треугольника создаем bounding box вручную
            sf::Vector2f position = triangleShape.getPosition();
            float radius = triangleShape.getPoint(2).y; // Основание треугольника
            return sf::FloatRect{
                position - sf::Vector2f(radius, radius), // position (левый верхний угол)
                sf::Vector2f(2 * radius, 2 * radius)     // size (ширина и высота)
            };
        }
        default:
            return sf::FloatRect{ sf::Vector2f(0.f, 0.f), sf::Vector2f(0.f, 0.f) };
        }
    }

    void draw(sf::RenderWindow& window) const {
        switch (shapeType) {
        case ShapeType::Circle:
            window.draw(circleShape);
            break;
        case ShapeType::Rectangle:
            window.draw(rectShape);
            break;
        case ShapeType::Triangle:
            window.draw(triangleShape);
            break;
        }
    }
};

class Simulation {
private:
    sf::RenderWindow window;
    std::vector<Body> bodies;
    bool isSimulationRunning = false;
    bool isAddingBody = false;
    bool isSettingMass = false;
    bool isSettingFriction = false;
    bool isSettingShape = false;
    ShapeType newBodyShape = ShapeType::Circle;
    sf::Vector2f newBodyPosition;
    float newBodyRadius = 20.f;
    float newBodyMass = 1.f;
    float newBodyFriction = 0.2f;

    // UI элементы
    sf::RectangleShape statusIndicator;
    sf::RectangleShape controlPanel;

    struct Button {
        sf::CircleShape shape;
        sf::Color activeColor;
        sf::Color inactiveColor;
        bool isActive = false;

        Button(float radius, sf::Vector2f position, sf::Color color)
            : shape(radius), activeColor(color), inactiveColor(color) {
            shape.setPosition(position);
            inactiveColor.a = 150; // Полупрозрачный когда неактивен
        }

        void setActive(bool active) {
            isActive = active;
            shape.setFillColor(active ? activeColor : inactiveColor);
        }

        bool contains(sf::Vector2f point) const {
            return shape.getGlobalBounds().contains(point);
        }
    };

    Button addButton;
    Button clearButton;
    Button playPauseButton;
    Button massButton;
    Button frictionButton;
    Button shapeButton;

    struct Slider {
        sf::RectangleShape track;
        sf::RectangleShape thumb;
        float minValue;
        float maxValue;
        float* linkedValue;

        Slider(sf::Vector2f position, sf::Vector2f size, float min, float max, float* value)
            : minValue(min), maxValue(max), linkedValue(value) {
            track.setSize(size);
            track.setPosition(position);
            track.setFillColor(sf::Color(100, 100, 100));

            thumb.setSize({ 5.f, size.y + 4.f });
            thumb.setFillColor(sf::Color::White);
            updateThumb();
        }

        void updateThumb() {
            float ratio = (*linkedValue - minValue) / (maxValue - minValue);
            thumb.setPosition({
                track.getPosition().x + ratio * track.getSize().x - 2.5f,
                track.getPosition().y - 2.f
                });
        }

        bool contains(sf::Vector2f point) const {
            return track.getGlobalBounds().contains(point);
        }

        void setFromMouseX(float mouseX) {
            float ratio = (mouseX - track.getPosition().x) / track.getSize().x;
            ratio = std::max(0.f, std::min(1.f, ratio));
            *linkedValue = minValue + ratio * (maxValue - minValue);
            updateThumb();
        }
    };

    Slider massSlider;
    Slider frictionSlider;

    // Формы для кнопки выбора формы
    std::array<sf::CircleShape, 3> shapeIcons;

public:
    Simulation() :
        window(sf::VideoMode({ 800, 600 }), "Physics Simulator"),
        addButton(15.f, { 20.f, 50.f }, sf::Color::Green),
        clearButton(15.f, { 60.f, 50.f }, sf::Color(255, 165, 0)),
        playPauseButton(15.f, { 100.f, 50.f }, sf::Color::Blue),
        massButton(15.f, { 140.f, 50.f }, sf::Color::Magenta),
        frictionButton(15.f, { 180.f, 50.f }, sf::Color::Cyan),
        shapeButton(15.f, { 220.f, 50.f }, sf::Color::Yellow),
        massSlider({ 260.f, 55.f }, { 150.f, 6.f }, 0.1f, 20.f, & newBodyMass),
        frictionSlider({ 260.f, 75.f }, { 150.f, 6.f }, 0.f, 1.f, & newBodyFriction)
    {
        // Настройка панели состояния
        statusIndicator.setSize({ 200.f, 20.f });
        statusIndicator.setPosition({ 10.f, 10.f });
        statusIndicator.setFillColor(sf::Color::Red);

        // Настройка панели управления
        controlPanel.setSize({ 420.f, 90.f });
        controlPanel.setPosition({ 5.f, 5.f });
        controlPanel.setFillColor(sf::Color(0, 0, 0, 150));
        controlPanel.setOutlineThickness(2.f);
        controlPanel.setOutlineColor(sf::Color::White);

        // Инициализация иконок форм
        shapeIcons[0] = sf::CircleShape(10.f); // Круг
        shapeIcons[0].setPosition({ 220.f, 40.f });
        shapeIcons[0].setFillColor(sf::Color::Yellow);

        shapeIcons[1] = sf::CircleShape(10.f, 4); // Квадрат (как 4-угольник)
        shapeIcons[1].setPosition({220.f, 40.f});
        shapeIcons[1].setFillColor(sf::Color::Yellow);

        shapeIcons[2] = sf::CircleShape(10.f, 3); // Треугольник
        shapeIcons[2].setPosition({ 220.f, 40.f });
        shapeIcons[2].setFillColor(sf::Color::Yellow);

        // Создаем пол
        createBody(ShapeType::Rectangle, 400.f, 550.f, 200.f, 10.f, sf::Color::Blue, 0.0f, 0.0f, true);
    }

    void run() {
        sf::Clock clock;
        while (window.isOpen()) {
            float dt = clock.restart().asSeconds();
            handleEvents();
            update(dt);
            render();
        }
    }

private:
    void handleEvents() {
        while (std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }

            event->visit([this](auto&& event) {
                using T = std::decay_t<decltype(event)>;

                if constexpr (std::is_same_v<T, sf::Event::MouseButtonPressed>) {
                    sf::Vector2f mousePos(event.position);

                    if (addButton.contains(mousePos)) {
                        isAddingBody = true;
                        isSettingMass = false;
                        isSettingFriction = false;
                        isSettingShape = false;
                        newBodyPosition = { 100.f, 100.f };
                    }
                    else if (clearButton.contains(mousePos)) {
                        bodies.clear();
                        createBody(ShapeType::Rectangle, 400.f, 550.f, 200.f, 10.f, sf::Color::Blue, 0.0f, 0.0f, true);
                    }
                    else if (playPauseButton.contains(mousePos)) {
                        isSimulationRunning = !isSimulationRunning;
                        statusIndicator.setFillColor(isSimulationRunning ? sf::Color::Green : sf::Color::Red);
                    }
                    else if (massButton.contains(mousePos)) {
                        isSettingMass = !isSettingMass;
                        isSettingFriction = false;
                        isAddingBody = false;
                        isSettingShape = false;
                        massButton.setActive(isSettingMass);
                        frictionButton.setActive(false);
                        shapeButton.setActive(false);
                    }
                    else if (frictionButton.contains(mousePos)) {
                        isSettingFriction = !isSettingFriction;
                        isSettingMass = false;
                        isAddingBody = false;
                        isSettingShape = false;
                        frictionButton.setActive(isSettingFriction);
                        massButton.setActive(false);
                        shapeButton.setActive(false);
                    }
                    else if (shapeButton.contains(mousePos)) {
                        isSettingShape = !isSettingShape;
                        isSettingMass = false;
                        isSettingFriction = false;
                        isAddingBody = false;
                        shapeButton.setActive(isSettingShape);
                        massButton.setActive(false);
                        frictionButton.setActive(false);

                        if (isSettingShape) {
                            // Циклически меняем форму
                            switch (newBodyShape) {
                            case ShapeType::Circle: newBodyShape = ShapeType::Rectangle; break;
                            case ShapeType::Rectangle: newBodyShape = ShapeType::Triangle; break;
                            case ShapeType::Triangle: newBodyShape = ShapeType::Circle; break;
                            }
                        }
                    }
                    else if (isAddingBody && event.button == sf::Mouse::Button::Left) {
                        newBodyPosition = sf::Vector2f(event.position);
                        createBody(newBodyShape, newBodyPosition.x, newBodyPosition.y,
                            newBodyRadius, newBodyRadius,
                            sf::Color(rand() % 200 + 55, rand() % 200 + 55, rand() % 200 + 55),
                            newBodyMass, newBodyFriction);
                        isAddingBody = false;
                    }
                    else if (isSettingMass && massSlider.contains(mousePos)) {
                        massSlider.setFromMouseX(mousePos.x);
                    }
                    else if (isSettingFriction && frictionSlider.contains(mousePos)) {
                        frictionSlider.setFromMouseX(mousePos.x);
                    }
                }

                if constexpr (std::is_same_v<T, sf::Event::MouseMoved>) {
                    if (isAddingBody) {
                        newBodyRadius = std::max(10.f, std::abs(event.position.y - newBodyPosition.y));
                    }
                    else if (isSettingMass && sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
                        massSlider.setFromMouseX(event.position.x);
                    }
                    else if (isSettingFriction && sf::Mouse::isButtonPressed(sf::Mouse::Button::Left)) {
                        frictionSlider.setFromMouseX(event.position.x);
                    }
                }

                if constexpr (std::is_same_v<T, sf::Event::KeyPressed>) {
                    if (event.code == sf::Keyboard::Key::Space) {
                        isSimulationRunning = !isSimulationRunning;
                        statusIndicator.setFillColor(isSimulationRunning ? sf::Color::Green : sf::Color::Red);
                    }
                    else if (event.code == sf::Keyboard::Key::A) {
                        isAddingBody = true;
                        isSettingMass = false;
                        isSettingFriction = false;
                        isSettingShape = false;
                        newBodyPosition = { 100.f, 100.f };
                    }
                    else if (event.code == sf::Keyboard::Key::C) {
                        bodies.clear();
                        createBody(ShapeType::Rectangle, 400.f, 550.f, 200.f, 10.f, sf::Color::Blue, 0.0f, 0.0f, true);
                    }
                    else if (event.code == sf::Keyboard::Key::M) {
                        isSettingMass = !isSettingMass;
                        isSettingFriction = false;
                        isAddingBody = false;
                        isSettingShape = false;
                        massButton.setActive(isSettingMass);
                        frictionButton.setActive(false);
                        shapeButton.setActive(false);
                    }
                    else if (event.code == sf::Keyboard::Key::F) {
                        isSettingFriction = !isSettingFriction;
                        isSettingMass = false;
                        isAddingBody = false;
                        isSettingShape = false;
                        frictionButton.setActive(isSettingFriction);
                        massButton.setActive(false);
                        shapeButton.setActive(false);
                    }
                    else if (event.code == sf::Keyboard::Key::S) {
                        isSettingShape = !isSettingShape;
                        isSettingMass = false;
                        isSettingFriction = false;
                        isAddingBody = false;
                        shapeButton.setActive(isSettingShape);
                        massButton.setActive(false);
                        frictionButton.setActive(false);

                        if (isSettingShape) {
                            // Циклически меняем форму
                            switch (newBodyShape) {
                            case ShapeType::Circle: newBodyShape = ShapeType::Rectangle; break;
                            case ShapeType::Rectangle: newBodyShape = ShapeType::Triangle; break;
                            case ShapeType::Triangle: newBodyShape = ShapeType::Circle; break;
                            }
                        }
                    }
                }
                });
        }
    }

    void update(float dt) {
        if (!isSimulationRunning) return;

        for (auto& body : bodies) body.update(dt);
        handleCollisions();

        for (auto& body : bodies) {
            if (body.isStatic) continue;

            sf::Vector2f pos = body.getPosition();
            auto bounds = body.getBoundingBox();
            float halfWidth = bounds.size.x / 2.f;  
            float halfHeight = bounds.size.y / 2.f; 
            if (pos.y + bounds.size.y / 2 > 550.f) {
                pos.y = 550.f - bounds.size.y / 2;
                body.setPosition(pos);
                body.velocity.y = -body.velocity.y * ELASTICITY;
            }
            if (pos.x - bounds.size.x / 2 < 0.f) {
                pos.x = bounds.size.x / 2;
                body.setPosition(pos);
                body.velocity.x = -body.velocity.x * ELASTICITY;
            }
            else if (pos.x + bounds.size.x / 2 > 800.f) {
                pos.x = 800.f - bounds.size.x / 2;
                body.setPosition(pos);
                body.velocity.x = -body.velocity.x * ELASTICITY;
            }
        }
    }
    bool rectsIntersect(const sf::FloatRect& a, const sf::FloatRect& b) {
        return !(a.position.x + a.size.x < b.position.x ||
            b.position.x + b.size.x < a.position.x ||
            a.position.y + a.size.y < b.position.y ||
            b.position.y + b.size.y < a.position.y);
    }
    void resolveCollision(Body& a, Body& b, const sf::Vector2f& normal, float penetration) {
        // Корректировка позиций
        sf::Vector2f correction = normal * penetration; // Небольшой запас


        if (!a.isStatic && !b.isStatic) {
            a.move(-correction * 0.5f);
            b.move(correction * 0.5f);
        }
        else if (a.isStatic) {
            b.move(correction);
        }
        else {
            a.move(-correction);
        }

        // Расчет относительной скорости
        sf::Vector2f relativeVel = b.velocity - a.velocity;
        float velAlongNormal = relativeVel.x * normal.x + relativeVel.y * normal.y;

        // Если объекты удаляются друг от друга - не обрабатываем
        if (velAlongNormal > 0) return;

        // Расчет импульса
        float restitution = ELASTICITY;
        float j = -(1 + restitution) * velAlongNormal;
        j /= (a.isStatic ? 0 : 1.0f / a.mass) + (b.isStatic ? 0 : 1.0f / b.mass);

        // Применение импульса
        sf::Vector2f impulse = j * normal;
        if (!a.isStatic) a.velocity -= impulse / a.mass;
        if (!b.isStatic) b.velocity += impulse / b.mass;
    }

    float calculatePenetration(const Body& a, const Body& b, const sf::Vector2f& normal) {
        // Для кругов
        if (a.shapeType == ShapeType::Circle && b.shapeType == ShapeType::Circle) {
            float distance = std::hypot(a.getPosition().x - b.getPosition().x,
                a.getPosition().y - b.getPosition().y);
            return (a.radius + b.radius) - distance;
        }

        // Для других комбинаций
        auto boundsA = a.getBoundingBox();
        auto boundsB = b.getBoundingBox();

        // Рассчитываем перекрытие по осям
        float overlapX = (boundsA.size.x + boundsB.size.x) * 0.5f -
            std::abs(a.getPosition().x - b.getPosition().x);
        float overlapY = (boundsA.size.y + boundsB.size.y) * 0.5f -
            std::abs(a.getPosition().y - b.getPosition().y);

        // Выбираем минимальное перекрытие как глубину проникновения
        return std::min(overlapX, overlapY);
    }
    // Получение вершин треугольника в мировых координатах
    std::array<sf::Vector2f, 3> getTrianglePoints(const Body& triangle) {
        std::array<sf::Vector2f, 3> points;
        const auto& transform = triangle.triangleShape.getTransform();
        for (int i = 0; i < 3; ++i) {
            points[i] = transform.transformPoint(triangle.triangleShape.getPoint(i));
        }
        return points;
    }

    // Получение нормалей ребер треугольника
    std::array<sf::Vector2f, 3> getTriangleNormals(const std::array<sf::Vector2f, 3>& points) {
        std::array<sf::Vector2f, 3> normals;
        for (int i = 0; i < 3; ++i) {
            sf::Vector2f edge = points[(i + 1) % 3] - points[i];
            normals[i] = sf::Vector2f(-edge.y, edge.x);
            float length = std::hypot(normals[i].x, normals[i].y);
            if (length > 0) normals[i] /= length;
        }
        return normals;
    }
    Projection project(const std::array<sf::Vector2f, 3>& points, const sf::Vector2f& axis) {
        float min = points[0].x * axis.x + points[0].y * axis.y;
        float max = min;

        for (int i = 1; i < 3; ++i) {
            float p = points[i].x * axis.x + points[i].y * axis.y;
            if (p < min) min = p;
            if (p > max) max = p;
        }

        return { min, max };
    }

    Projection project(const sf::FloatRect& rect, const sf::Vector2f& axis) {
        std::array<sf::Vector2f, 4> points = {
            rect.position,
            rect.position + sf::Vector2f(rect.size.x, 0),
            rect.position + rect.size,
            rect.position + sf::Vector2f(0, rect.size.y)
        };

        float min = points[0].x * axis.x + points[0].y * axis.y;
        float max = min;

        for (const auto& p : points) {
            float projection = p.x * axis.x + p.y * axis.y;
            min = std::min(min, projection);
            max = std::max(max, projection);
        }

        return { min, max };
    }
    bool checkTriangleCollision(const Body& triangle, const Body& other) {
        auto triPoints = getTrianglePoints(triangle);
        auto normals = getTriangleNormals(triPoints);

        // Проверка по нормалям треугольника
        for (const auto& normal : normals) {
            Projection p1 = project(triPoints, normal);
            Projection p2 = other.shapeType == ShapeType::Circle ?
                projectCircle(other.getPosition(), other.radius, normal) :
                project(other.getBoundingBox(), normal);

            if (!p1.overlaps(p2)) return false;
        }

        // Проверка по нормалям другого объекта
        if (other.shapeType == ShapeType::Rectangle) {
            std::array<sf::Vector2f, 2> rectAxes = {
                sf::Vector2f(1, 0),
                sf::Vector2f(0, 1)
            };

            for (const auto& axis : rectAxes) {
                Projection p1 = project(triPoints, axis);
                Projection p2 = project(other.getBoundingBox(), axis);

                if (!p1.overlaps(p2)) return false;
            }
        }

        return true;
    }

    Projection projectCircle(const sf::Vector2f& center, float radius, const sf::Vector2f& axis) {
        float projection = center.x * axis.x + center.y * axis.y;
        return { projection - radius, projection + radius };
    }

    sf::Vector2f calculateTriangleMTD(const Body& triangle, const Body& other) {
        auto triPoints = getTrianglePoints(triangle);
        auto normals = getTriangleNormals(triPoints);

        sf::Vector2f mtd;
        float minOverlap = std::numeric_limits<float>::max();

        // Проверка по нормалям треугольника
        for (const auto& normal : normals) {
            Projection p1 = project(triPoints, normal);
            Projection p2 = other.shapeType == ShapeType::Circle ?
                projectCircle(other.getPosition(), other.radius, normal) :
                project(other.getBoundingBox(), normal);

            if (!p1.overlaps(p2)) return sf::Vector2f(0, 0);

            float overlap = std::min(p1.max - p2.min, p2.max - p1.min);
            if (overlap < minOverlap) {
                minOverlap = overlap;
                mtd = normal * overlap;
            }
        }

        // Проверка по осям другого объекта
        if (other.shapeType == ShapeType::Rectangle) {
            std::array<sf::Vector2f, 2> rectAxes = {
                sf::Vector2f(1, 0),
                sf::Vector2f(0, 1)
            };

            for (const auto& axis : rectAxes) {
                Projection p1 = project(triPoints, axis);
                Projection p2 = project(other.getBoundingBox(), axis);

                if (!p1.overlaps(p2)) return sf::Vector2f(0, 0);

                float overlap = std::min(p1.max - p2.min, p2.max - p1.min);
                if (overlap < minOverlap) {
                    minOverlap = overlap;
                    mtd = axis * overlap;
                }
            }
        }

        return mtd;
    }

    // Новая функция проверки коллизий между разными формами
    bool checkCollision(const Body& a, const Body& b) {
        const auto& boundsA = a.getBoundingBox();
        const auto& boundsB = b.getBoundingBox();

        // Проверка отсутствия пересечения по осям
        if (boundsA.position.x + boundsA.size.x < boundsB.position.x ||
            boundsB.position.x + boundsB.size.x < boundsA.position.x ||
            boundsA.position.y + boundsA.size.y < boundsB.position.y ||
            boundsB.position.y + boundsB.size.y < boundsA.position.y) {
            return false;
        }

        if (a.shapeType == ShapeType::Triangle || b.shapeType == ShapeType::Triangle) {
            const Body& triangle = (a.shapeType == ShapeType::Triangle) ? a : b;
            const Body& other = (a.shapeType == ShapeType::Triangle) ? b : a;
            return checkTriangleCollision(triangle, other);
        }

        // Специальная обработка для кругов
        if (a.shapeType == ShapeType::Circle || b.shapeType == ShapeType::Circle) {
            sf::Vector2f circlePos = (a.shapeType == ShapeType::Circle) ? a.getPosition() : b.getPosition();
            float radius = (a.shapeType == ShapeType::Circle) ? a.radius : b.radius;
            sf::Vector2f otherPos = (a.shapeType == ShapeType::Circle) ? b.getPosition() : a.getPosition();

            if (b.shapeType == ShapeType::Rectangle || a.shapeType == ShapeType::Rectangle) {
                // Проверка столкновения круга с прямоугольником
                sf::Vector2f closestPoint;
                closestPoint.x = std::max(boundsB.position.x, std::min(circlePos.x, boundsB.position.x + boundsB.size.x));
                closestPoint.y = std::max(boundsB.position.y, std::min(circlePos.y, boundsB.position.y + boundsB.size.y));

                float distance = std::hypot(circlePos.x - closestPoint.x, circlePos.y - closestPoint.y);
                return distance < radius;
            }
        }
        return true;
    }

    // Обновленный метод handleCollisions()
    void handleCollisions() {
        for (size_t i = 0; i < bodies.size(); ++i) {
            for (size_t j = i + 1; j < bodies.size(); ++j) {
                Body& body1 = bodies[i];
                Body& body2 = bodies[j];

                if (body1.isStatic && body2.isStatic) continue;

                if (checkCollision(body1, body2)) {
                    // Расчет направления столкновения
                    sf::Vector2f dir = body2.getPosition() - body1.getPosition();
                    float distance = std::max(0.1f, std::hypot(dir.x, dir.y));
                    sf::Vector2f normal = dir / distance;

                    // Расчет глубины проникновения
                    float penetration = calculatePenetration(body1, body2, normal);
                    if (penetration <= 0) continue;

                    // Разрешение столкновения
                    resolveCollision(body1, body2, normal, penetration);
                }
            }
        }
    }

    void render() {
        window.clear(sf::Color::Black);

        // Рисуем физические тела
        for (const auto& body : bodies) {
            body.draw(window);
        }

        // Рисуем UI панель
        window.draw(controlPanel);
        window.draw(statusIndicator);

        // Кнопки
        window.draw(addButton.shape);
        window.draw(clearButton.shape);
        window.draw(playPauseButton.shape);
        window.draw(massButton.shape);
        window.draw(frictionButton.shape);
        window.draw(shapeButton.shape);

        // Слайдеры
        window.draw(massSlider.track);
        window.draw(massSlider.thumb);
        window.draw(frictionSlider.track);
        window.draw(frictionSlider.thumb);

        // Подписи к слайдерам
        sf::RectangleShape massLabel({ 50.f, 10.f });
        massLabel.setPosition({ 260.f, 40.f });
        massLabel.setFillColor(sf::Color::Magenta);
        window.draw(massLabel);

        sf::RectangleShape frictionLabel({ 50.f, 10.f });
        frictionLabel.setPosition({ 260.f, 60.f });
        frictionLabel.setFillColor(sf::Color::Cyan);
        window.draw(frictionLabel);

        // Иконка текущей формы
        switch (newBodyShape) {
        case ShapeType::Circle: window.draw(shapeIcons[0]); break;
        case ShapeType::Rectangle: window.draw(shapeIcons[1]); break;
        case ShapeType::Triangle: window.draw(shapeIcons[2]); break;
        }

        // Индикатор добавления нового тела
        if (isAddingBody) {
            sf::Color previewColor(255, 255, 255, 100);
            switch (newBodyShape) {
            case ShapeType::Circle: {
                sf::CircleShape preview(newBodyRadius);
                preview.setPosition(newBodyPosition);
                preview.setOrigin({ newBodyRadius, newBodyRadius });
                preview.setFillColor(previewColor);
                window.draw(preview);
                break;
            }
            case ShapeType::Rectangle: {
                sf::RectangleShape preview({ newBodyRadius * 2, newBodyRadius * 2 });
                preview.setPosition(newBodyPosition);
                preview.setOrigin({ newBodyRadius, newBodyRadius });
                preview.setFillColor(previewColor);
                window.draw(preview);
                break;
            }
            case ShapeType::Triangle: {
                sf::ConvexShape preview;
                preview.setPointCount(3);
                preview.setPoint(0, { 0, -newBodyRadius });
                preview.setPoint(1, { newBodyRadius, newBodyRadius });
                preview.setPoint(2, { -newBodyRadius, newBodyRadius });
                preview.setPosition(newBodyPosition);
                preview.setFillColor(previewColor);
                window.draw(preview);
                break;
            }
            }
        }

        window.display();
    }

    void createBody(ShapeType type, float x, float y, float width, float height, sf::Color color,
        float mass = 1.f, float friction = 0.2f, bool isStatic = false) {
        float size = std::min(width, height) / 2.f;
        bodies.emplace_back(type, size, sf::Vector2f(x, y), color, mass, friction, isStatic);
    }
};

int main() {
    Simulation simulation;
    simulation.run();
    return 0;
}